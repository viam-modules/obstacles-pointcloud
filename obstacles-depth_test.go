package obstaclespointcloud

import (
	"context"
	"image"
	"testing"

	"github.com/golang/geo/r3"

	"github.com/pkg/errors"
	"go.viam.com/test"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils"
	"go.viam.com/rdk/vision/detection3d"
)

func TestObstaclesDepthRegistration(t *testing.T) {
	r := &inject.Robot{}
	r.LoggerFunc = func() logging.Logger { return nil }
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context, _ map[string]interface{}) (pc.PointCloud, error) {
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

// rgbdCamera is a fake RGB-D camera looking straight down: it serves a 400 x 400 color image
// first and then a depth image at half that resolution of a table 500 mm away, with a box
// standing 40 mm proud of it, about 60 mm square and 20 mm off-centre in x.
func rgbdCamera(withDepth bool) *inject.Camera {
	const w, h = 400, 400
	cam := inject.NewCamera("rgbd")
	cam.PropertiesFunc = func(context.Context) (camera.Properties, error) {
		return camera.Properties{IntrinsicParams: &transform.PinholeCameraIntrinsics{
			Width: w, Height: h, Fx: 400, Fy: 400, Ppx: w / 2, Ppy: h / 2,
		}}, nil
	}
	cam.ImagesFunc = func(context.Context, []string, map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
		color, err := camera.NamedImageFromImage(image.NewRGBA(image.Rect(0, 0, w, h)), "color", utils.MimeTypeJPEG, data.Annotations{})
		if err != nil {
			return nil, resource.ResponseMetadata{}, err
		}
		images := []camera.NamedImage{color}
		if withDepth {
			dm := rimage.NewEmptyDepthMap(w/2, h/2)
			for y := 0; y < h/2; y++ {
				for x := 0; x < w/2; x++ {
					d := rimage.Depth(500)
					if x >= 96 && x < 122 && y >= 87 && y < 113 {
						d = 460
					}
					dm.Set(x, y, d)
				}
			}
			depth, err := camera.NamedImageFromImage(dm, "depth", utils.MimeTypeRawDepth, data.Annotations{})
			if err != nil {
				return nil, resource.ResponseMetadata{}, err
			}
			images = append(images, depth)
		}
		return images, resource.ResponseMetadata{}, nil
	}
	return cam
}

func TestDepthMapFromCameraSkipsColor(t *testing.T) {
	dm, err := depthMapFromCamera(context.Background(), rgbdCamera(true))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, dm.Width(), test.ShouldEqual, 200)
	test.That(t, dm.GetDepth(100, 100), test.ShouldEqual, rimage.Depth(460))

	_, err = depthMapFromCamera(context.Background(), rgbdCamera(false))
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "no depth image from camera")
	test.That(t, err.Error(), test.ShouldContainSubstring, `image "color"`)
}

func TestObstaclesDepthFindsBoxOnTableFromRGBDCamera(t *testing.T) {
	cfg := &ErCCLConfig{
		MinPtsInPlane:    2000,
		MinPtsInSegment:  100,
		MaxDistFromPlane: 10,
		NormalVec:        r3.Vector{X: 0, Y: 0, Z: 1},
	}
	cfg.SetDefaultValues()
	segmenter := (&obsDepth{clusteringConf: cfg}).buildObsDepth(logging.NewTestLogger(t))
	objects, err := segmenter(context.Background(), rgbdCamera(true))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(objects), test.ShouldEqual, 1)
	center := objects[0].Geometry.Pose().Point()
	// the box spans x 96..121 px, y 87..112 px of the half-resolution depth image (principal
	// point 100, 100 and f 200 px there): x (108.5 - 100) / 200 * 460 = 20 mm, y 0 mm. Without
	// scaling the intrinsics to the depth image, x would be (108.5 - 200) / 400 * 460 = -105 mm.
	test.That(t, center.X, test.ShouldAlmostEqual, 20, 5)
	test.That(t, center.Y, test.ShouldAlmostEqual, 0, 5)
	test.That(t, center.Z, test.ShouldAlmostEqual, 460, 5)

	// GetDetections3D builds an octree from each object's points: depth-map points carry no data
	detections, err := detection3d.FromSegmenter("depth", segmenter)(context.Background(), rgbdCamera(true))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(detections), test.ShouldEqual, 1)
	test.That(t, len(detections[0].Transforms), test.ShouldEqual, 2)
}

func TestObstaclesDepthGroundNormal(t *testing.T) {
	deps := resource.Dependencies{camera.Named("rgbd"): rgbdCamera(true)}
	conf := &ObsDepthConfig{DefaultCamera: "rgbd"}
	test.That(t, conf.groundNormal(), test.ShouldResemble, r3.Vector{X: 0, Y: -1, Z: 0})
	conf.GroundPlaneNormalVec = NormalVec{Z: 1}
	test.That(t, conf.groundNormal(), test.ShouldResemble, r3.Vector{X: 0, Y: 0, Z: 1})
	conf.GroundPlaneNormalVec = NormalVec{Z: -2}
	test.That(t, conf.groundNormal(), test.ShouldResemble, r3.Vector{X: 0, Y: 0, Z: -1})
	_, err := registerObstaclesDepth(context.Background(), vision.Named("depth"), conf, deps, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
}

func TestSameView(t *testing.T) {
	in := &transform.PinholeCameraIntrinsics{Width: 1280, Height: 720}
	test.That(t, sameView(in, 640, 360), test.ShouldBeTrue)
	test.That(t, sameView(in, 848, 480), test.ShouldBeTrue) // a RealSense depth size: rounding
	test.That(t, sameView(in, 640, 480), test.ShouldBeFalse)
	test.That(t, sameView(&transform.PinholeCameraIntrinsics{}, 640, 480), test.ShouldBeTrue)
}

func TestObstaclesDepthWarnsWhenDepthIsAnotherView(t *testing.T) {
	cam := rgbdCamera(true) // depth 200 x 200
	cam.PropertiesFunc = func(context.Context) (camera.Properties, error) {
		return camera.Properties{IntrinsicParams: &transform.PinholeCameraIntrinsics{
			Width: 400, Height: 300, Fx: 400, Fy: 400, Ppx: 200, Ppy: 150,
		}}, nil
	}
	cfg := &ErCCLConfig{MinPtsInPlane: 2000, MinPtsInSegment: 100, MaxDistFromPlane: 10, NormalVec: r3.Vector{Z: 1}}
	cfg.SetDefaultValues()
	logger, logs := logging.NewObservedTestLogger(t)
	_, err := (&obsDepth{clusteringConf: cfg}).buildObsDepth(logger)(context.Background(), cam)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, logs.FilterMessageSnippet("aspect ratio differs").Len(), test.ShouldEqual, 1)

	logger, logs = logging.NewObservedTestLogger(t) // the same view at half resolution: no warning
	_, err = (&obsDepth{clusteringConf: cfg}).buildObsDepth(logger)(context.Background(), rgbdCamera(true))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, logs.FilterMessageSnippet("aspect ratio differs").Len(), test.ShouldEqual, 0)
}
