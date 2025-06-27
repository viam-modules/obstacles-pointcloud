// Package obstaclespointcloud uses an underlying camera that provides point clouds to fulfill GetObjectPointClouds,
// applying a point cloud clustering algorithm to find distinct obstacles.
// The RDK version of this service is buggy and should be further investigated at some point. This implements
// the same functionality, and demonstrates the same buggy and laggy behavior as the RDK version.
package obstaclespointcloud

import (
	"context"
	"image"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	vision "go.viam.com/rdk/services/vision"
	vis "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	objdet "go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/segmentation"
	"go.viam.com/rdk/vision/viscapture"
	"go.viam.com/utils/rpc"
)

var Model = resource.NewModel("viam", "vision", "obstacles-pointcloud")
var errUnimplemented = errors.New("unimplemented")

type PointCloudConfig struct {
	resource.TriviallyValidateConfig
	DefaultCamera            string  `json:"camera_name"`
	MinPointsInPlane         int     `json:"min_points_in_plane"`
	MinPointsInSegment       int     `json:"min_points_in_segment"`
	MaxDistFromPlaneMM       float64 `json:"max_dist_from_plane_mm"`
	GroundAngleToleranceDegs float64 `json:"ground_angle_tolerance_degs"`
	ClusteringRadius         int     `json:"clustering_radius"`
	ClusteringStrictness     float64 `json:"clustering_strictness"`
}

type obstaclePointCloudService struct {
	resource.AlwaysRebuild
	name           resource.Name
	logger         logging.Logger
	deps           resource.Dependencies
	defaultCamera  camera.Camera
	clusteringConf *segmentation.ErCCLConfig
}

func init() {
	resource.RegisterService(vision.API, Model, resource.Registration[vision.Service, *PointCloudConfig]{
		Constructor: func(
			ctx context.Context,
			deps resource.Dependencies,
			conf resource.Config,
			logger logging.Logger,
		) (vision.Service, error) {
			attrs, err := resource.NativeConfig[*PointCloudConfig](conf)
			if err != nil {
				return nil, err
			}
			return registerObstaclePointCloudService(ctx, conf.ResourceName(), attrs, deps, logger)
		},
	})
}

// Validate ensures all parts of the config are valid and specifies dependencies.
func (cfg *PointCloudConfig) Validate(path string) ([]string, []string, error) {
	var reqDeps []string
	var optDeps []string

	if cfg.DefaultCamera == "" {
		return nil, nil, errors.New("a 'camera_name' is required for the obstacles-pointcloud service")
	}

	reqDeps = append(reqDeps, cfg.DefaultCamera)
	return reqDeps, optDeps, nil
}

// registerObstaclePointCloudService creates a new 3D clustering segmenter from the config.
func registerObstaclePointCloudService(
	ctx context.Context,
	name resource.Name,
	conf *PointCloudConfig,
	deps resource.Dependencies,
	logger logging.Logger,
) (vision.Service, error) {
	_, span := trace.StartSpan(ctx, "service::vision::registerObstaclePointCloudService")
	defer span.End()

	// build the clustering config
	clusteringConf := &segmentation.ErCCLConfig{
		MinPtsInPlane:        conf.MinPointsInPlane,
		MinPtsInSegment:      conf.MinPointsInSegment,
		MaxDistFromPlane:     conf.MaxDistFromPlaneMM,
		NormalVec:            r3.Vector{X: 0, Y: -1, Z: 0},
		AngleTolerance:       conf.GroundAngleToleranceDegs,
		ClusteringRadius:     conf.ClusteringRadius,
		ClusteringStrictness: conf.ClusteringStrictness,
	}
	err := clusteringConf.CheckValid()
	if err != nil {
		return nil, errors.Wrap(err, "error building clustering config for obstacles-pointcloud")
	}

	var defaultCam camera.Camera
	if conf.DefaultCamera != "" {
		defaultCam, err = camera.FromDependencies(deps, conf.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
	}

	myObsPointCloud := &obstaclePointCloudService{
		name:           name,
		logger:         logger,
		deps:           deps,
		defaultCamera:  defaultCam,
		clusteringConf: clusteringConf,
	}
	return myObsPointCloud, nil
}

func (s *obstaclePointCloudService) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vis.Object, error) {
	cam, err := s.getCamera(cameraName)
	if err != nil {
		return nil, err
	}

	pc, err := cam.NextPointCloud(ctx)
	if err != nil {
		return nil, errors.Wrap(err, "failed to get next point cloud from camera")
	}

	return segmentation.ApplyERCCLToPointCloud(ctx, pc, s.clusteringConf)
}

func (s *obstaclePointCloudService) getCamera(cameraName string) (camera.Camera, error) {
	if cameraName != "" {
		cam, err := camera.FromDependencies(s.deps, cameraName)
		if err != nil {
			return nil, errors.Wrapf(err, "could not get camera %q from dependencies", cameraName)
		}
		return cam, nil
	}
	if s.defaultCamera != nil {
		return s.defaultCamera, nil
	}
	return nil, errors.New("a camera name was not specified and no default_camera is configured")
}

func (s *obstaclePointCloudService) CaptureAllFromCamera(ctx context.Context, cameraName string, captureOptions viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
	cam, err := s.getCamera(cameraName)
	if err != nil {
		return viscapture.VisCapture{}, err
	}

	result := viscapture.VisCapture{}

	if captureOptions.ReturnImage {
		img, err := camera.DecodeImageFromCamera(ctx, "", nil, cam)
		if err != nil {
			return viscapture.VisCapture{}, err
		}
		result.Image = img
	}

	if captureOptions.ReturnObject {
		objects, err := s.GetObjectPointClouds(ctx, cameraName, extra)
		if err != nil {
			return viscapture.VisCapture{}, err
		}
		result.Objects = objects
	}

	result.Detections = []objdet.Detection{}
	result.Classifications = classification.Classifications{}

	return result, nil
}

func (s *obstaclePointCloudService) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
	return &vision.Properties{
		ClassificationSupported: false,
		DetectionSupported:      false,
		ObjectPCDsSupported:     true,
	}, nil
}

func (s *obstaclePointCloudService) Name() resource.Name {
	return s.name
}

func (s *obstaclePointCloudService) Close(context.Context) error {
	return nil
}

func (s *obstaclePointCloudService) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (vision.Service, error) {
	return nil, errUnimplemented
}

func (s *obstaclePointCloudService) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, errUnimplemented
}

func (s *obstaclePointCloudService) Detections(ctx context.Context, img image.Image, extra map[string]interface{}) ([]objdet.Detection, error) {
	return nil, errUnimplemented
}

func (s *obstaclePointCloudService) ClassificationsFromCamera(ctx context.Context, cameraName string, n int, extra map[string]interface{}) (classification.Classifications, error) {
	return nil, errUnimplemented
}

func (s *obstaclePointCloudService) Classifications(ctx context.Context, img image.Image, n int, extra map[string]interface{}) (classification.Classifications, error) {
	return nil, errUnimplemented
}

func (s *obstaclePointCloudService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, errUnimplemented
}
