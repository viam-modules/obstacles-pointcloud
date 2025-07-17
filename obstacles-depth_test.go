package obstaclespointcloud

import (
	"context"
	"testing"

	"github.com/pkg/errors"
	"go.viam.com/test"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/testutils/inject"
)

func TestObstaclesDepthRegistration(t *testing.T) {
	r := &inject.Robot{}
	r.LoggerFunc = func() logging.Logger { return nil }
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pc.PointCloud, error) {
		return nil, errors.New("no properties")
	}
	r.ResourceNamesFunc = func() []resource.Name {
		return []resource.Name{camera.Named("fakeCamera")}
	}
	r.ResourceByNameFunc = func(n resource.Name) (resource.Resource, error) {
		switch n.Name {
		case "fakeCamera":
			return cam, nil
		default:
			return nil, resource.NewNotFoundError(n)
		}
	}

	// Create dependencies map
	deps := make(resource.Dependencies)
	deps[camera.Named("fakeCamera")] = cam

	params := &ObsDepthConfig{
		MinPtsInPlane:        100,
		MaxDistFromPlane:     10,
		MinPtsInSegment:      3,
		AngleTolerance:       20,
		ClusteringRadius:     5,
		ClusteringStrictness: 3,
		DefaultCamera:        "fakeCamera",
	}
	name := vision.Named("test_obs_depth")
	// bad registration, no parameters
	_, err := registerObstaclesDepth(context.Background(), name, nil, deps, nil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "cannot be nil")
	// bad registration, parameters out of bounds
	params.ClusteringRadius = -3
	_, err = registerObstaclesDepth(context.Background(), name, params, deps, nil)
	test.That(t, err, test.ShouldBeNil)
	// successful registration
	params.ClusteringRadius = 1
	service, err := registerObstaclesDepth(context.Background(), name, params, deps, nil)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, service.Name(), test.ShouldResemble, name)
	// successful registration, valid default camera
	params.DefaultCamera = "fakeCamera"
	_, err = registerObstaclesDepth(context.Background(), name, params, deps, nil)
	test.That(t, err, test.ShouldBeNil)
	// successful registration, invalid default camera
	params.DefaultCamera = "not-camera"
	_, err = registerObstaclesDepth(context.Background(), name, params, deps, nil)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "could not find camera \"not-camera\"")
}
