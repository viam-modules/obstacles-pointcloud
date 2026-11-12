# Obstacles PointCloud Module

This module provides two vision service models that find well-separated objects above a flat surface (a floor or a table). Both find the biggest plane in the camera's point cloud, remove it, and cluster the remaining points into objects:

- `viam:obstacles-pointcloud:obstacles-pointcloud` uses the camera's point cloud (`NextPointCloud`).
- `viam:obstacles-depth:obstacles-depth` builds the point cloud itself from the camera's depth image and intrinsic parameters. If the camera has no intrinsic parameters, it instead returns a single point at the median depth of the image: how far away the scene in front of the camera is.

Both serve `GetObjectPointClouds` and, from 0.2.0, `GetDetections3D`. Each detection is a box around one object's points, in the camera's frame, with the object's points attached. Neither serves 2D detections or classifications.

## Model viam:obstacles-pointcloud:obstacles-pointcloud

### Configuration

```json
{
  "camera_name": "camera-1",
  "min_points_in_plane": 500,
  "min_points_in_segment": 10,
  "max_dist_from_plane_mm": 100,
  "ground_plane_normal_vec": { "x": 0, "y": 0, "z": 1 },
  "ground_angle_tolerance_degs": 30,
  "clustering_radius": 5,
  "clustering_strictness": 1
}
```

### Attributes

| Name | Type | Inclusion | Default | Description |
| --- | --- | --- | --- | --- |
| `camera_name` | string | **Required** | | The camera to use for calls that don't name one. It must serve point clouds. |
| `min_points_in_plane` | int | Optional | `500` | How many points a plane needs to count as the ground. This tells large planes, like floors, walls and tables, from small ones, like the tops of bottle caps. |
| `min_points_in_segment` | int | Optional | `10` | Objects with fewer points are dropped. |
| `max_dist_from_plane_mm` | float | Optional | `100` | How far above and below the ideal ground plane points still count as the plane and are removed. Use a high value for uneven ground such as tall grass; for objects on a table, use less than the objects' height. |
| `ground_plane_normal_vec` | object `{x, y, z}` | Optional | `{"x": 0, "y": 0, "z": 1}` | The ground plane's normal, in the camera's frame. Cameras differ: a lidar's ground normal is usually `+z`, `{"x": 0, "y": 0, "z": 1}`; a forward-looking RealSense's `+z` points out of the lens, so its ground normal is `{"x": 0, "y": -1, "z": 0}`. Must be a unit vector; anything else falls back to the default. |
| `ground_angle_tolerance_degs` | float | Optional | `30` | How far, in degrees, the plane found may tilt from `ground_plane_normal_vec` and still count as the ground, for example on slopes. |
| `clustering_radius` | int | Optional | `5` | Which neighbouring points are close enough to join the same cluster. A small radius may split one object into several; a large one may merge objects that are close together. |
| `clustering_strictness` | float | Optional | `1` | The threshold for sorting neighbouring points into the same cluster. Higher values make many small clusters; lower values make fewer, bigger ones. |

Zero, the value of an omitted attribute, means the default.

### Example

```json
{
  "name": "objects",
  "api": "rdk:service:vision",
  "model": "viam:obstacles-pointcloud:obstacles-pointcloud",
  "attributes": {
    "camera_name": "camera-1",
    "min_points_in_plane": 2000,
    "min_points_in_segment": 100,
    "max_dist_from_plane_mm": 10
  }
}
```

## Model viam:obstacles-depth:obstacles-depth

### Configuration

```json
{
  "camera_name": "camera-1",
  "min_points_in_plane": 500,
  "min_points_in_segment": 10,
  "max_dist_from_plane_mm": 100,
  "ground_angle_tolerance_degs": 30,
  "clustering_radius": 5,
  "clustering_strictness": 1
}
```

### Attributes

`camera_name`, `min_points_in_plane`, `min_points_in_segment`, `max_dist_from_plane_mm`, `ground_angle_tolerance_degs`, `clustering_radius` and `clustering_strictness` work as for `obstacles-pointcloud` above, with the same defaults.

This model has no `ground_plane_normal_vec`: the ground normal is fixed at `{"x": 0, "y": -1, "z": 0}`, the camera's `-y` axis, which suits a forward-looking camera such as a RealSense on a rover.

### Limits

- The camera's first image must be its depth image: the model decodes the first image the camera returns. Cameras that serve a color image first, such as a RealSense with `"sensors": ["color", "depth"]`, fail with `could not convert image to depth map`.
- Because the ground normal is fixed, the model doesn't suit a camera looking down at a table.

### Example

```json
{
  "name": "depth-objects",
  "api": "rdk:service:vision",
  "model": "viam:obstacles-depth:obstacles-depth",
  "attributes": {
    "camera_name": "camera-1",
    "min_points_in_segment": 10,
    "max_dist_from_plane_mm": 100
  }
}
```

## Example camera configuration

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
    "sensors": ["depth", "color"]
  }
}
```

## Getting object poses in the world

Configure the camera's frame in the [frame system](https://docs.viam.com/operate/reference/services/frame-system/#configuration). The objects and detections come back in the camera's frame; move them to the world with the frame system, for example `TransformPose` on a detection's pose.
