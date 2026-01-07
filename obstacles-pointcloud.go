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

var ObstaclesPointCloud = resource.NewModel("viam", "obstacles-pointcloud", "obstacles-pointcloud")

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

type NormalVec struct {
	X float64 `json:"x,omitempty"`
	Y float64 `json:"y,omitempty"`
	Z float64 `json:"z,omitempty"`
}

type ObstaclesPointCloudConfig struct {
	MinPtsInPlane        int       `json:"min_points_in_plane"`
	MinPtsInSegment      int       `json:"min_points_in_segment"`
	MaxDistFromPlane     float64   `json:"max_dist_from_plane_mm"`
	ClusteringRadius     int       `json:"clustering_radius"`
	ClusteringStrictness float64   `json:"clustering_strictness"`
	AngleTolerance       float64   `json:"ground_angle_tolerance_degs"`
	DefaultCamera        string    `json:"camera_name"`
	GroundPlaneNormalVec NormalVec `json:"ground_plane_normal_vec"`
}

func (cfg *ObstaclesPointCloudConfig) Validate(path string) ([]string, []string, error) {
	var deps []string
	var optionalDeps []string
	if cfg.DefaultCamera == "" {
		return nil, optionalDeps, errors.Errorf(`expected "camera_name" attribute (DefaultCamera) for obstacles pointcloud at %q`, path)
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

	var groundPlaneNormalVec r3.Vector
	if conf.GroundPlaneNormalVec.X == 0 && conf.GroundPlaneNormalVec.Y == 0 && conf.GroundPlaneNormalVec.Z == 0 {
		groundPlaneNormalVec = r3.Vector{X: 0, Y: 0, Z: 1}
	} else {
		groundPlaneNormalVec = r3.Vector{X: conf.GroundPlaneNormalVec.X, Y: conf.GroundPlaneNormalVec.Y, Z: conf.GroundPlaneNormalVec.Z}
	}
	// build the clustering config
	cfg := &ErCCLConfig{
		MinPtsInPlane:        conf.MinPtsInPlane,
		MinPtsInSegment:      conf.MinPtsInSegment,
		MaxDistFromPlane:     conf.MaxDistFromPlane,
		NormalVec:            groundPlaneNormalVec,
		AngleTolerance:       conf.AngleTolerance,
		ClusteringRadius:     conf.ClusteringRadius,
		ClusteringStrictness: conf.ClusteringStrictness,
		DefaultCamera:        conf.DefaultCamera,
	}
	cfg.SetDefaultValues()
	if conf.DefaultCamera != "" {
		_, err := camera.FromProvider(deps, conf.DefaultCamera)
		if err != nil {
			return nil, errors.Errorf("could not find camera %q", conf.DefaultCamera)
		}
	}
	segmenter := segmentation.Segmenter(cfg.ErCCLAlgorithm)
	return vision.NewService(name, deps, logger, nil, nil, nil, segmenter, conf.DefaultCamera)
}
