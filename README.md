# Obstacles PointCloud Module
This module provides two services:
- `obstacles-pointcloud`: identifies well separated objects above a flat plane. It first identifies the biggest plane in the scene, eliminates that plane, and clusters the remaining points into objects.
- `obstacles-depth`: measures the depth of an object in a 3D point cloud.

### Configuration
The following attribute template can be used to configure both services:

```json
{
  "min_points_in_plane": 500,
  "min_points_in_segment": 10,
  "max_dist_from_plane_mm": 100,
  "ground_angle_tolerance_degs": 30,
  "clustering_radius": 1,
  "clustering_strictness": 5,
  "camera_name": "camera-1"
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `min_points_in_plane` | int  | Optional  | Minimum number of points in the plane |
| `min_points_in_segment` | int | Optional  | Minimum number of points in a segment |
| `max_dist_from_plane_mm` | float | Optional  | Maximum distance from the plane in mm |
| `ground_angle_tolerance_degs` | float | Optional  | Angle tolerance for the ground plane |
| `clustering_radius` | int | Optional  | Clustering radius |
| `clustering_strictness` | float | Optional  | Clustering strictness |
| `camera_name` | string | Optional  | Name of the default camera |

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

#### Example Module Configuration for `obstacles-pointcloud`

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
    "camera_name": "camera-1"
  }
}
```

#### Example Module Configuration for `obstacles-depth`

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
    "camera_name": "camera-1"
  }
}
```
