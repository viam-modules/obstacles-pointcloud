// Package obstaclespointcloud uses the 3D radius clustering algorithm as defined in the
// RDK vision/segmentation package as vision model.
package obstaclespointcloud

import (
	"context"
	"fmt"
	"sort"
	"strings"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/depthadapter"
	"go.viam.com/rdk/rimage/transform"
	svision "go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
	vision "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/detection3d"
)

var ObstaclesDepth = resource.NewModel("viam", "obstacles-depth", "obstacles-depth")

func init() {
	resource.RegisterService(svision.API, ObstaclesDepth, resource.Registration[svision.Service, *ObsDepthConfig]{
		Constructor: func(
			ctx context.Context, deps resource.Dependencies, c resource.Config, logger logging.Logger,
		) (svision.Service, error) {
			attrs, err := resource.NativeConfig[*ObsDepthConfig](c)
			if err != nil {
				return nil, err
			}
			return registerObstaclesDepth(ctx, c.ResourceName(), attrs, deps, logger)
		},
	})
}

// ObsDepthConfig specifies the parameters to be used for the obstacle depth service.
type ObsDepthConfig struct {
	MinPtsInPlane        int       `json:"min_points_in_plane"`
	MinPtsInSegment      int       `json:"min_points_in_segment"`
	MaxDistFromPlane     float64   `json:"max_dist_from_plane_mm"`
	ClusteringRadius     int       `json:"clustering_radius"`
	ClusteringStrictness float64   `json:"clustering_strictness"`
	AngleTolerance       float64   `json:"ground_angle_tolerance_degs"`
	DefaultCamera        string    `json:"camera_name"`
	GroundPlaneNormalVec NormalVec `json:"ground_plane_normal_vec"`
}

// groundNormal is the configured ground normal scaled to unit length, or obsDepthDefaultNormal
// when none is set.
func (cfg *ObsDepthConfig) groundNormal() r3.Vector {
	if n := cfg.GroundPlaneNormalVec; n.X != 0 || n.Y != 0 || n.Z != 0 {
		return r3.Vector{X: n.X, Y: n.Y, Z: n.Z}.Normalize()
	}
	return obsDepthDefaultNormal
}

// obsDepthDefaultNormal is the ground normal when none is configured: the camera's -y axis,
// as for a forward-looking camera.
var obsDepthDefaultNormal = r3.Vector{X: 0, Y: -1, Z: 0}

// obsDepth is the underlying struct actually used by the service.
type obsDepth struct {
	clusteringConf *ErCCLConfig
}

func (cfg *ObsDepthConfig) Validate(path string) ([]string, []string, error) {
	var deps []string
	var optionalDeps []string
	if cfg.DefaultCamera == "" {
		return nil, optionalDeps, errors.Errorf(`expected "camera_name" attribute (DefaultCamera) for obstacles depth at %q`, path)
	}
	deps = append(deps, cfg.DefaultCamera)

	if cfg.MinPtsInPlane < 0 {
		return nil, optionalDeps, errors.New("min_points_in_plane must be positive")
	}

	if cfg.MinPtsInSegment < 0 {
		return nil, optionalDeps, errors.New("min_points_in_segment must be positive")
	}

	if cfg.MaxDistFromPlane < 0 {
		return nil, optionalDeps, errors.New("max_dist_from_plane_mm must be positive")
	}

	if cfg.ClusteringRadius < 0 {
		return nil, optionalDeps, errors.New("clustering_radius must be positive")
	}

	if cfg.ClusteringStrictness < 0 {
		return nil, optionalDeps, errors.New("clustering_strictness must be non-negative")
	}

	if cfg.AngleTolerance < 0 {
		return nil, optionalDeps, errors.New("ground_angle_tolerance_degs must be non-negative")
	}

	return deps, optionalDeps, nil
}

func registerObstaclesDepth(
	ctx context.Context,
	name resource.Name,
	conf *ObsDepthConfig,
	deps resource.Dependencies,
	logger logging.Logger,
) (svision.Service, error) {
	_, span := trace.StartSpan(ctx, "service::vision::registerObstacleDepth")
	defer span.End()
	if conf == nil {
		return nil, errors.New("config for obstacles_depth cannot be nil")
	}
	// build the clustering config
	cfg := &ErCCLConfig{
		MinPtsInPlane:        conf.MinPtsInPlane,
		MinPtsInSegment:      conf.MinPtsInSegment,
		MaxDistFromPlane:     conf.MaxDistFromPlane,
		NormalVec:            conf.groundNormal(),
		AngleTolerance:       conf.AngleTolerance,
		ClusteringRadius:     conf.ClusteringRadius,
		ClusteringStrictness: conf.ClusteringStrictness,
	}
	cfg.SetDefaultValues()
	myObsDep := &obsDepth{
		clusteringConf: cfg,
	}
	if conf.DefaultCamera != "" {
		_, err := camera.FromProvider(deps, conf.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
	}

	segmenter := myObsDep.buildObsDepth(logger) // does the thing
	return svision.NewService(name, deps, logger, nil, nil, nil, detection3d.FromSegmenter(name.ShortName(), segmenter), conf.DefaultCamera)
}

// BuildObsDepth will check for intrinsics and determine how to build based on that.
func (o *obsDepth) buildObsDepth(logger logging.Logger) func(
	ctx context.Context, src camera.Camera) ([]*vision.Object, error) {
	return func(ctx context.Context, src camera.Camera) ([]*vision.Object, error) {
		props, err := src.Properties(ctx)
		if err != nil {
			logger.CWarnw(ctx, "could not find camera properties. obstacles depth started without camera's intrinsic parameters", "error", err)
			return o.obsDepthNoIntrinsics(ctx, src)
		}
		if props.IntrinsicParams == nil {
			logger.CWarn(ctx, "obstacles depth started but camera did not have intrinsic parameters")
			return o.obsDepthNoIntrinsics(ctx, src)
		}
		return o.obsDepthWithIntrinsics(ctx, src, props.IntrinsicParams)
	}
}

// buildObsDepthNoIntrinsics will return the median depth in the depth map as a Geometry point.
func (o *obsDepth) obsDepthNoIntrinsics(ctx context.Context, src camera.Camera) ([]*vision.Object, error) {
	dm, err := depthMapFromCamera(ctx, src)
	if err != nil {
		return nil, err
	}
	depData := dm.Data()
	if len(depData) == 0 {
		return nil, errors.New("could not get info from depth map")
	}
	// Sort the depth data [smallest...largest]
	sort.Slice(depData, func(i, j int) bool {
		return depData[i] < depData[j]
	})
	med := int(0.5 * float64(len(depData)))
	pt := spatialmath.NewPoint(r3.Vector{X: 0, Y: 0, Z: float64(depData[med])}, "")
	toReturn := make([]*vision.Object, 1)
	toReturn[0] = &vision.Object{Geometry: pt}
	return toReturn, nil
}

// buildObsDepthWithIntrinsics will use the methodology in Manduchi et al. to find obstacle points
// before clustering and projecting those points into 3D obstacles.
func (o *obsDepth) obsDepthWithIntrinsics(
	ctx context.Context, src camera.Camera, intrinsics *transform.PinholeCameraIntrinsics,
) ([]*vision.Object, error) {
	dm, err := depthMapFromCamera(ctx, src)
	if err != nil {
		return nil, err
	}
	cloud := depthadapter.ToPointCloud(dm, scaledIntrinsics(intrinsics, dm.Width(), dm.Height()))
	return ApplyERCCLToPointCloud(ctx, cloud, o.clusteringConf)
}

// depthMapFromCamera returns the camera's depth image as a depth map. RGB-D cameras serve
// several images, often color first, so it tries the images whose MIME type is raw depth
// first, then the rest in order, and returns the first one that converts.
func depthMapFromCamera(ctx context.Context, src camera.Camera) (*rimage.DepthMap, error) {
	images, _, err := src.Images(ctx, nil, nil)
	if err != nil {
		return nil, errors.Wrapf(err, "could not get images from %s", src.Name())
	}
	if len(images) == 0 {
		return nil, errors.Errorf("camera %s returned no images", src.Name())
	}
	sort.SliceStable(images, func(i, j int) bool {
		return images[i].MimeType() == utils.MimeTypeRawDepth && images[j].MimeType() != utils.MimeTypeRawDepth
	})
	var reasons []string
	for _, ni := range images {
		img, err := ni.Image(ctx)
		if err == nil {
			var dm *rimage.DepthMap
			if dm, err = rimage.ConvertImageToDepthMap(ctx, img); err == nil {
				return dm, nil
			}
		}
		reasons = append(reasons, fmt.Sprintf("image %q (%s): %v", ni.SourceName, ni.MimeType(), err))
	}
	return nil, errors.Errorf("no depth image from camera %s: %s", src.Name(), strings.Join(reasons, "; "))
}

// scaledIntrinsics returns the intrinsics for an image of width x height. Cameras report the
// color image's intrinsics, and some serve depth at another resolution of the same view.
func scaledIntrinsics(in *transform.PinholeCameraIntrinsics, width, height int) *transform.PinholeCameraIntrinsics {
	if in.Width <= 0 || in.Height <= 0 || (in.Width == width && in.Height == height) {
		return in
	}
	sx, sy := float64(width)/float64(in.Width), float64(height)/float64(in.Height)
	return &transform.PinholeCameraIntrinsics{
		Width: width, Height: height,
		Fx: in.Fx * sx, Fy: in.Fy * sy,
		Ppx: in.Ppx * sx, Ppy: in.Ppy * sy,
	}
}
