// Package obstaclespointcloud uses the 3D radius clustering algorithm as defined in the
// RDK vision/segmentation package as vision model.
package obstaclespointcloud

import (
	"context"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/vision/segmentation"
)

var ObstaclesPointCloud = resource.NewModel("viam", "vision", "obstacles-pointcloud")

func init() {
	resource.RegisterService(vision.API, ObstaclesPointCloud, resource.Registration[vision.Service, *ObstaclesPointCloudConfig]{
	    Constructor: func(
			ctx context.Context, deps resource.Dependencies, c resource.Config, logger logging.Logger,
		) (vision.Service, error) {
			attrs, err := resource.NativeConfig[*ObstaclesPointCloudConfig](c)
			if err != nil {
				return nil, err
			}
			return registerPointCloudSegmenter(ctx, c.ResourceName(), attrs, deps, logger)
		},
	})
}

type ObstaclesPointCloudConfig struct {
	MinPtsInPlane        int     `json:"min_points_in_plane"`
	MinPtsInSegment      int     `json:"min_points_in_segment"`
	MaxDistFromPlane     float64 `json:"max_dist_from_plane_mm"`
	ClusteringRadius     int     `json:"clustering_radius"`
	ClusteringStrictness float64 `json:"clustering_strictness"`
	AngleTolerance       float64 `json:"ground_angle_tolerance_degs"`
	DefaultCamera        string  `json:"camera_name"`
	NormalVec            r3.Vector `json:"ground_plane_normal_vec"`
}

func (cfg *ObstaclesPointCloudConfig) Validate(path string) ([]string, []string, error) {
	var deps []string
	var warnings []string
	if cfg.DefaultCamera == "" {
		return nil, warnings, errors.Errorf(`expected "camera_name" attribute (DefaultCamera) for obstacles pointcloud at %q`, path)
	}
	deps = append(deps, cfg.DefaultCamera)

	if cfg.MinPtsInPlane <= 0 {
		return nil, warnings, errors.New("min_points_in_plane must be positive")
	}
	if cfg.MinPtsInSegment <= 0 {
		return nil, warnings, errors.New("min_points_in_segment must be positive")
	}
	if cfg.MaxDistFromPlane <= 0 {
		return nil, warnings, errors.New("max_dist_from_plane_mm must be positive")
	}
	if cfg.ClusteringRadius <= 0 {
		return nil, warnings, errors.New("clustering_radius must be positive")
	}
	if cfg.ClusteringStrictness < 0 {
		return nil, warnings, errors.New("clustering_strictness must be non-negative")
	}
	if cfg.AngleTolerance < 0 {
		return nil, warnings, errors.New("ground_angle_tolerance_degs must be non-negative")
	}
	if cfg.NormalVec == (r3.Vector{}) {
		return nil, warnings, errors.New("ground_plane_normal_vec must be set")
	}
	return deps, warnings, nil
}

// registerOPSegmenter creates a new 3D radius clustering segmenter from the config.
func registerPointCloudSegmenter(
	ctx context.Context,
	name resource.Name,
	conf *ObstaclesPointCloudConfig,
	deps resource.Dependencies,
	logger logging.Logger,
) (vision.Service, error) {
	_, span := trace.StartSpan(ctx, "service::vision::registerObstaclesPointcloud")
	defer span.End()
	if conf == nil {
		return nil, errors.New("config for obstacles pointcloud segmenter cannot be nil")
	}
	// build the clustering config
	cfg := &segmentation.ErCCLConfig{
		MinPtsInPlane:        conf.MinPtsInPlane,
		MinPtsInSegment:      conf.MinPtsInSegment,
		MaxDistFromPlane:     conf.MaxDistFromPlane,
		NormalVec:            r3.Vector{0, -1, 0},
		AngleTolerance:       conf.AngleTolerance,
		ClusteringRadius:     conf.ClusteringRadius,
		ClusteringStrictness: conf.ClusteringStrictness,
		DefaultCamera:        conf.DefaultCamera,
	}
	err := cfg.CheckValid()
	if err != nil {
		return nil, errors.Wrap(err, "error building clustering config for obstacles pointcloud")
	}	
	if conf.DefaultCamera != "" {
		_, err := camera.FromDependencies(deps, conf.DefaultCamera)
		if err != nil {
			logger.Warn(errors.Wrap(err, "could not find camera"))
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
	}
	segmenter := segmentation.Segmenter(cfg.ErCCLAlgorithm)
	return vision.NewService(name, deps, logger, nil, nil, nil, segmenter, conf.DefaultCamera)
}
