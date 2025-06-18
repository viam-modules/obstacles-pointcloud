package obstaclespointcloud

import (
	"context"
	"image"

	"github.com/pkg/errors"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	vis "go.viam.com/rdk/vision"
	"go.viam.com/rdk/vision/classification"
	objdet "go.viam.com/rdk/vision/objectdetection"
	"go.viam.com/rdk/vision/viscapture"
	"go.viam.com/utils/rpc"
)

var (
	ObstaclesPointcloud = resource.NewModel("viam", "obstacles-pointcloud", "obstacles-pointcloud")
	errUnimplemented    = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(vision.API, ObstaclesPointcloud,
		resource.Registration[vision.Service, *Config]{
			Constructor: newObstaclesPointcloudObstaclesPointcloud,
		},
	)
}

type Config struct {
	/*
		Put config attributes here. There should be public/exported fields
		with a `json` parameter at the end of each attribute.

		Example config struct:
			type Config struct {
				Pin   string `json:"pin"`
				Board string `json:"board"`
				MinDeg *float64 `json:"min_angle_deg,omitempty"`
			}

		If your model does not need a config, replace *Config in the init
		function with resource.NoNativeConfig
	*/
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	// Add config validation code here
	return nil, nil, nil
}

type obstaclesPointcloudObstaclesPointcloud struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()
}

func newObstaclesPointcloudObstaclesPointcloud(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (vision.Service, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewObstaclesPointcloud(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewObstaclesPointcloud(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (vision.Service, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &obstaclesPointcloudObstaclesPointcloud{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *obstaclesPointcloudObstaclesPointcloud) Name() resource.Name {
	return s.name
}

func (s *obstaclesPointcloudObstaclesPointcloud) NewClientFromConn(ctx context.Context, conn rpc.ClientConn, remoteName string, name resource.Name, logger logging.Logger) (vision.Service, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) DetectionsFromCamera(ctx context.Context, cameraName string, extra map[string]interface{}) ([]objdet.Detection, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) Detections(ctx context.Context, img image.Image, extra map[string]interface{}) ([]objdet.Detection, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) ClassificationsFromCamera(ctx context.Context, cameraName string, n int, extra map[string]interface{}) (classification.Classifications, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) Classifications(ctx context.Context, img image.Image, n int, extra map[string]interface{}) (classification.Classifications, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) GetObjectPointClouds(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*vis.Object, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) GetProperties(ctx context.Context, extra map[string]interface{}) (*vision.Properties, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) CaptureAllFromCamera(ctx context.Context, cameraName string, captureOptions viscapture.CaptureOptions, extra map[string]interface{}) (viscapture.VisCapture, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	panic("not implemented")
}

func (s *obstaclesPointcloudObstaclesPointcloud) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}
