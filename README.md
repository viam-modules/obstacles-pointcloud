# Obstacles PointCloud Module
This module builds a segmenter that identifies well separated objects above a flat plane. It first identifies the biggest plane in the scene, eliminates that plane, and clusters the remaining points into objects

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "min_points_in_plane": 500,
  "min_points_in_segment": 10,
  "max_dist_from_plane_mm": 100,
  "ground_angle_tolerance_degs": 30,
  "clustering_radius": 1,
  "clustering_strictness": 5,
  "camera_name": "camera-1",
  "ground_plane_normal_vec": {
    "x": 0,
    "y": -1,
    "z": 0
  }
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `min_points_in_plane` | int  | Required  | Minimum number of points in the plane |
| `min_points_in_segment` | int | Optional  | Minimum number of points in a segment |
| `max_dist_from_plane_mm` | float | Optional  | Maximum distance from the plane in mm |
| `ground_angle_tolerance_degs` | float | Optional  | Angle tolerance for the ground plane |
| `clustering_radius` | int | Optional  | Clustering radius |
| `clustering_strictness` | float | Optional  | Clustering strictness |
| `camera_name` | string | Optional  | Camera name |
| `ground_plane_normal_vec` | vector | Optional  | Ground plane normal vector. Default is (0, -1, 0) |

#### Example Camera Configuration

```json
{
  "name": "camera-1",
  "api": "rdk:component:camera",
  "model": "viam:camera:realsense",
  "attributes": {
    "width_px": 640,
    "height_px": 480,
    "little_endian_depth": false,
    "serial_number": "",
    "sensors": [
      "depth",
      "color"
    ]
  }
}
```

#### Example Module Configuration

```json
{
  "name": "vision-1",
  "api": "rdk:service:vision",
  "model": "viam:vision:obstacles-pointcloud",
  "attributes": {
    "min_points_in_segment": 10,
    "max_dist_from_plane_mm": 100,
    "ground_angle_tolerance_degs": 30,
    "clustering_radius": 1,
    "clustering_strictness": 5,
    "min_points_in_plane": 500,
    "ground_plane_normal_vec": {
      "x": 0,
      "y": -1,
      "z": 0
    },
    "camera_name": "camera-1"
  }
}
```

# Obstacles Depth Module

This module provides a service for reading depth images from a depth camera and detecting obstacles in the environment by projecting them onto a point cloud and applying a point cloud clustering algorithm.

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "min_points_in_plane": 500,
  "min_points_in_segment": 10,
  "max_dist_from_plane_mm": 100,
  "ground_angle_tolerance_degs": 30,
  "clustering_radius": 1,
  "clustering_strictness": 5,
  "camera_name": "camera-1",
  "ground_plane_normal_vec": {
    "x": 0,
    "y": -1,
    "z": 0
  }
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `min_points_in_plane` | int  | Required  | Minimum number of points in the plane |
| `min_points_in_segment` | int | Optional  | Minimum number of points in a segment |
| `max_dist_from_plane_mm` | float | Optional  | Maximum distance from the plane in mm |
| `ground_angle_tolerance_degs` | float | Optional  | Angle tolerance for the ground plane |
| `clustering_radius` | int | Optional  | Clustering radius |
| `clustering_strictness` | float | Optional  | Clustering strictness |
| `camera_name` | string | Optional  | Camera name |
| `ground_plane_normal_vec` | vector | Optional  | Ground plane normal vector. Default is (0, -1, 0) |

#### Example Camera Configuration

```json
{
  "name": "camera-1",
  "api": "rdk:component:camera",
  "model": "viam:camera:realsense",
  "attributes": {
    "width_px": 640,
    "height_px": 480,
    "little_endian_depth": false,
    "serial_number": "",
    "sensors": [
      "depth",
      "color"
    ]
  }
}
```

#### Example Module Configuration

```json
{
  "name": "vision-1",
  "api": "rdk:service:vision",
  "model": "viam:vision:obstacles-depth",
  "attributes": {
    "min_points_in_segment": 10,
    "max_dist_from_plane_mm": 100,
    "ground_angle_tolerance_degs": 30,
    "clustering_radius": 1,
    "clustering_strictness": 5,
    "min_points_in_plane": 500,
    "ground_plane_normal_vec": {
      "x": 0,
      "y": -1,
      "z": 0
    },
    "camera_name": "camera-1"
  }
}
```
