# Obstacles PointCloud Module
This module provides two services:
- `obstacles-pointcloud`: identifies well separated objects above a flat plane. It first identifies the biggest plane in the scene, eliminates that plane, and clusters the remaining points into objects.
- `obstacles-depth`: identify well separated objects above a flat plane. It measures the depth of an object in a 3D point cloud.

### Configuration
In your vision service's panel, fill in the attributes field for your service:

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
| `camera_name` | string | **Required** | The default camera to use for calls to `GetObjectPointClouds`. |
| `min_points_in_plane` | int  | Optional  | An integer that specifies how many points to put on the flat surface or ground plane when clustering. This is to distinguish between large planes, like the floors and walls, and small planes, like the tops of bottle caps. <br> Default: `500` </br> |
| `min_points_in_segment` | int | Optional  | An integer that sets a minimum size to the returned objects, and filters out all other found objects below that size. <br> Default: `10` </br> |
| `max_dist_from_plane_mm` | float | Optional  | A float that determines how much area above and below an ideal ground plane should count as the plane for which points are removed. For fields with tall grass, this should be a high number. The default value is 100 mm. <br> Default: `100` </br> |
| `ground_plane_normal_vec` | (int, int, int) | Optional | A `(x,y,z)` vector that represents the normal vector of the ground plane. Different cameras have different coordinate systems. For example, a lidar's ground plane will point in the `+z` direction `(0, 0, 1)`. On the other hand, the intel realsense `+z` direction points out of the camera lens, and its ground plane is in the negative y direction `(0, -1, 0)`. <br> Default: `(0, 0, 1)` </br> |
| `ground_angle_tolerance_degs` | float | Optional  | An integer that determines how strictly the found ground plane should match the `ground_plane_normal_vec`. For example, even if the ideal ground plane is purely flat, a rover may encounter slopes and hills. The algorithm should find a ground plane even if the found plane is at a slant, up to a certain point. <br> Default: `30` </br> |
| `clustering_radius` | int | Optional  | An integer that specifies which neighboring points count as being "close enough" to be potentially put in the same cluster. This parameter determines how big the candidate clusters should be, or, how many points should be put on a flat surface. A small clustering radius is likely to split different parts of a large cluster into distinct objects. A large clustering radius is likely to aggregate closely spaced clusters into one object. <br> Default: `1` </br> |
| `clustering_strictness` | float | Optional  | An integer that determines the probability threshold for sorting neighboring points into the same cluster, or how "easy" `viam-server` should determine it is to sort the points the machine's camera sees into this pointcloud. When the `clustering_radius` determines the size of the candidate clusters, then the clustering_strictness determines whether the candidates will count as a cluster. If `clustering_strictness` is set to a large value, many small clusters are likely to be made, rather than a few big clusters. The lower the number, the bigger your clusters will be. <br> Default: `5` </br> |

Click the **Save** button in the top right corner of the page and use the **Test** panel to test your service.

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

## FAQ

## Identify multiple boxes over the flat plane:

First, [configure your frame system]([/operate/reference/services/frame-system/#configuration](https://docs.viam.com/operate/reference/services/frame-system/#configuration)) to configure the relative spatial orientation of the components of your machine, including your camera, within Viam's frame system. After configuring your frame system, your camera will populate its own `Properties` with these spatial intrinsic parameters from the frame system. You can get those parameters from your camera through the [camera API](https://docs.viam.com/dev/reference/apis/components/camera/#getproperties).

The segmenter now returns multiple boxes within the `GeometryInFrame` object it captures.
