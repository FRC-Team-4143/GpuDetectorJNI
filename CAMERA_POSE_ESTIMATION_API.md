# Camera Pose Estimation using Perspective-n-Point (PnP)

## Overview

The Camera Pose Estimation module calculates the camera's position and orientation in 3D space using detected AprilTags and their known world positions. This uses the Perspective-n-Point (PnP) algorithm with lens distortion correction via OpenCV.

## Features

- **PnP Algorithm**: Solves for camera pose given 2D-3D correspondences
- **Lens Distortion Correction**: Accounts for radial and tangential distortion
- **Multi-Tag Support**: Uses multiple visible tags for improved accuracy
- **Tag Map Management**: Configure known tag positions and orientations
- **Flexible API**: C++ and JNI interfaces for integration

## How It Works

### The PnP Problem

Given:
- Known 3D positions of AprilTag corners in world coordinates
- Observed 2D positions of those corners in the camera image
- Camera intrinsic parameters (focal length, principal point)
- Lens distortion coefficients

Find:
- Camera position (x, y, z) in world frame
- Camera orientation (quaternion) in world frame

### Algorithm Steps

1. **Tag Map**: Store known AprilTag IDs with their world positions/orientations
2. **Detection**: Detect AprilTags in the camera image (corners in pixels)
3. **Correspondence**: Match detected tags to known tags in the map
4. **Corner Transform**: Calculate 3D world coordinates of tag corners
5. **PnP Solve**: Use OpenCV's `solvePnP` with lens distortion correction
6. **Result**: Camera pose (position + quaternion orientation)

### Multiple Tags

When multiple tags are visible:
- All corner correspondences are combined (4 corners × N tags = 4N points)
- PnP uses all points simultaneously for a more robust solution
- More tags generally means better accuracy and reliability
- OpenCV automatically selects appropriate solver (ITERATIVE for 6+ points, P3P for 4 points)

## Data Structures

### TagPose

Represents a known AprilTag's position and orientation in world frame:

```cpp
struct TagPose {
  int tag_id;             // Unique tag identifier
  double x, y, z;         // Position in meters (world frame)
  double qw, qx, qy, qz;  // Orientation (quaternion, world frame)
  double size;            // Tag size in meters (default: 0.1524m = 6 inches)
};
```

### CameraPose

Represents the estimated camera pose:

```cpp
struct CameraPose {
  double x, y, z;         // Position in meters
  double qw, qx, qy, qz;  // Orientation (quaternion)
  double error;           // Average reprojection error in pixels
};
```

## C++ API

### Creating an Estimator

```cpp
#include "frc971/orin/camera_pose_estimator.h"

// Camera intrinsic parameters
frc971::apriltag::CameraMatrix camera_matrix{
  800.0,  // fx - focal length x (pixels)
  320.0,  // cx - principal point x (pixels)
  800.0,  // fy - focal length y (pixels)
  240.0   // cy - principal point y (pixels)
};

// Lens distortion coefficients (k1, k2, p1, p2, k3)
frc971::apriltag::DistCoeffs dist_coeffs{
  -0.3,  // k1 - radial distortion
  0.1,   // k2 - radial distortion
  0.0,   // p1 - tangential distortion
  0.0,   // p2 - tangential distortion
  0.0    // k3 - radial distortion
};

frc971::apriltag::CameraPoseEstimator estimator(camera_matrix, dist_coeffs);
```

### Adding Tags to the Map

```cpp
// Add a tag at world origin, facing +Z, 6 inches (0.1524m) square
frc971::apriltag::TagPose tag0(
  0,                    // tag ID
  0.0, 0.0, 0.0,        // position (x, y, z in meters)
  1.0, 0.0, 0.0, 0.0,   // quaternion (identity = no rotation)
  0.1524                // size in meters
);
estimator.AddTagToMap(tag0);

// Add a tag 1 meter to the right
frc971::apriltag::TagPose tag1(
  1,                    // tag ID
  1.0, 0.0, 0.0,        // position
  1.0, 0.0, 0.0, 0.0,   // quaternion
  0.1524                // size
);
estimator.AddTagToMap(tag1);

// Remove a tag
estimator.RemoveTagFromMap(0);

// Clear all tags
estimator.ClearTagMap();
```

### Estimating Camera Pose

```cpp
// Get detections from GPU detector
const zarray_t* detections = gpu_detector->Detections();

// Estimate camera pose from all visible known tags
frc971::apriltag::CameraPose camera_pose;
bool success = estimator.EstimateCameraPose(detections, camera_pose);

if (success) {
  std::cout << "Camera at: [" << camera_pose.x << ", " 
            << camera_pose.y << ", " << camera_pose.z << "]\n";
  std::cout << "Orientation: [" << camera_pose.qw << ", "
            << camera_pose.qx << ", " << camera_pose.qy << ", "
            << camera_pose.qz << "]\n";
  std::cout << "Reprojection error: " << camera_pose.error << " pixels\n";
}
```

### With Initial Estimate

```cpp
// Provide initial estimate for faster convergence
frc971::apriltag::CameraPose initial_estimate;
initial_estimate.x = 0.5;
initial_estimate.y = 0.0;
initial_estimate.z = 1.0;
initial_estimate.qw = 1.0;

frc971::apriltag::CameraPose camera_pose;
bool success = estimator.EstimateCameraPose(detections, camera_pose, &initial_estimate);
```

## JNI API (Java Interface)

### Creating and Destroying Estimator

```java
// Create estimator with camera parameters
long estimatorHandle = GpuDetectorJNI.createCameraPoseEstimator(
    fx, cx, fy, cy,           // Camera matrix
    k1, k2, p1, p2, k3        // Distortion coefficients
);

// Destroy when done
GpuDetectorJNI.destroyCameraPoseEstimator(estimatorHandle);
```

### Managing Tag Map

```java
// Add a tag to the map
GpuDetectorJNI.addTagToMap(
    estimatorHandle,
    tagId,                    // int
    x, y, z,                  // double (position in meters)
    qw, qx, qy, qz,           // double (quaternion)
    size                      // double (tag size in meters)
);

// Remove a tag
GpuDetectorJNI.removeTagFromMap(estimatorHandle, tagId);

// Clear all tags
GpuDetectorJNI.clearTagMap(estimatorHandle);

// Get number of tags in map
int numTags = GpuDetectorJNI.getTagMapSize(estimatorHandle);
```

### Estimating Pose

```java
// Estimate camera pose from detected tags
double[] cameraPose = GpuDetectorJNI.estimateCameraPose(
    estimatorHandle,
    detectorHandle            // GPU detector handle with detections
);

if (cameraPose != null) {
    double x = cameraPose[0];
    double y = cameraPose[1];
    double z = cameraPose[2];
    double qw = cameraPose[3];
    double qx = cameraPose[4];
    double qy = cameraPose[5];
    double qz = cameraPose[6];
    double error = cameraPose[7];  // Reprojection error in pixels
}
```

## Complete Example

```cpp
#include "frc971/orin/camera_pose_estimator.h"
#include "frc971/orin/971apriltag.h"

// Setup
frc971::apriltag::CameraMatrix camera_matrix{800, 320, 800, 240};
frc971::apriltag::DistCoeffs dist_coeffs{-0.2, 0.05, 0, 0, 0};
frc971::apriltag::CameraPoseEstimator estimator(camera_matrix, dist_coeffs);

// Configure known tag positions (e.g., tags on a field)
// Tag 0 at origin
estimator.AddTagToMap(frc971::apriltag::TagPose(
    0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1524));

// Tag 1 at 1m to the right
estimator.AddTagToMap(frc971::apriltag::TagPose(
    1, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1524));

// Tag 2 at 1m up
estimator.AddTagToMap(frc971::apriltag::TagPose(
    2, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1524));

// During runtime - process frame and detect tags
gpu_detector->DetectGrayHost(image_data);
const zarray_t* detections = gpu_detector->Detections();

// Estimate camera pose
frc971::apriltag::CameraPose camera_pose;
if (estimator.EstimateCameraPose(detections, camera_pose)) {
    // Use camera pose for robot localization
    std::cout << "Robot camera at: [" << camera_pose.x << ", "
              << camera_pose.y << ", " << camera_pose.z << "]\n";
    
    // Low error means good quality estimate
    if (camera_pose.error < 1.0) {
        std::cout << "High quality pose estimate\n";
    }
}
```

## Coordinate Frames

### World Frame
- Fixed reference frame where tag positions are defined
- Example: Field corners, walls, or known landmarks

### Tag Frame
- Origin at tag center
- X-axis points right, Y-axis points up, Z-axis points out of tag
- Tag lies in XY plane (Z=0 for corners)

### Camera Frame
- Origin at camera optical center
- X-axis points right in image, Y-axis points down, Z-axis points forward
- Estimated pose transforms points from world to camera frame

## Calibration

### Camera Matrix Parameters

Get these from camera calibration:
- **fx, fy**: Focal length in pixels (typically 500-1500 for common cameras)
- **cx, cy**: Principal point (usually near image center)

### Distortion Coefficients

- **k1, k2, k3**: Radial distortion (barrel/pincushion effect)
- **p1, p2**: Tangential distortion (lens not parallel to sensor)

Use OpenCV's calibration tools or:
```bash
python -m opencv.calibrate
```

## Performance

- **Single tag**: ~0.1ms (P3P or IPPE solver)
- **Multiple tags**: ~0.2ms (ITERATIVE solver with refinement)
- **Accuracy**: Sub-centimeter with good calibration and multiple tags
- **Minimum**: Requires at least 1 visible tag (4 corners)
- **Optimal**: 3+ tags for robust estimation

## Error Handling

The estimator returns `false` if:
- No detections provided
- No detected tags are in the map
- Fewer than 4 point correspondences
- PnP solver fails (degenerate configuration)

Check `camera_pose.error`:
- **< 0.5 pixels**: Excellent
- **0.5-1.0 pixels**: Good
- **1.0-2.0 pixels**: Acceptable
- **> 2.0 pixels**: Poor (check calibration, lighting, or tag positions)

## Tips for Best Results

1. **Calibrate camera properly**: Use checkerboard calibration
2. **Use multiple tags**: 3+ tags significantly improves accuracy
3. **Good lighting**: Ensure tags are well-lit and in focus
4. **Tag size**: Larger tags = better detection at distance
5. **Tag placement**: Distribute tags around the area for coverage
6. **Reprojection error**: Monitor error to detect bad estimates
7. **Initial estimate**: Provide if camera moves smoothly (tracking)

## Dependencies

- **OpenCV**: Core, Calib3D modules
- **AprilTag**: Detection library (already included)
- **C++20**: For modern features

## Build Requirements

```cmake
find_package(OpenCV REQUIRED)
target_link_libraries(yourlib ${OpenCV_LIBS})
```

Requires OpenCV 4.0+ for best PnP solvers.
