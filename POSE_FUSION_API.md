# Multi-Camera Pose Fusion API

## Overview

The Multi-Camera Pose Fusion module provides functionality to combine pose estimates from multiple cameras into a single, robust pose estimation. This is particularly useful for robotic applications where multiple cameras observe the same AprilTags from different viewpoints.

## Features

- **Weighted Averaging**: Combines multiple pose estimates using confidence-based weighting
- **Time Synchronization**: Filters poses within a configurable time window
- **Camera Offset Support**: Accounts for each camera's physical offset from the robot center
- **Flexible Buffer Management**: Configurable buffer size for storing recent localizations
- **Quaternion-based Orientation**: Uses unit quaternions for robust orientation representation

## Data Structures

### Pose3D

Represents a 3D pose with position and orientation:

```cpp
struct Pose3D {
  double x, y, z;           // Position in meters
  double qw, qx, qy, qz;    // Orientation as unit quaternion
};
```

### CameraLocalization

Represents a single camera's pose estimate with metadata:

```cpp
struct CameraLocalization {
  Pose3D pose;              // The pose estimate
  double confidence;        // Confidence factor (0.0 to 1.0)
  std::chrono::nanoseconds timestamp;  // Timestamp
  Pose3D camera_offset;     // Camera offset from robot center
  int camera_id;            // Camera identifier
};
```

## C++ API

### Creating a Pose Fusion Object

```cpp
#include "frc971/orin/pose_fusion.h"

frc971::apriltag::PoseFusion fusion;
```

### Adding Camera Localizations

```cpp
frc971::apriltag::CameraLocalization loc;
loc.camera_id = 0;
loc.timestamp = std::chrono::nanoseconds(timestamp);
loc.pose.x = 1.0;
loc.pose.y = 2.0;
loc.pose.z = 3.0;
loc.pose.qw = 1.0;  // Identity quaternion
loc.pose.qx = 0.0;
loc.pose.qy = 0.0;
loc.pose.qz = 0.0;
loc.confidence = 0.95;

// Set camera offset if needed
loc.camera_offset.x = 0.1;
loc.camera_offset.y = 0.0;
loc.camera_offset.z = 0.0;

fusion.AddCameraLocalization(loc);
```

### Fusing Poses

```cpp
frc971::apriltag::Pose3D fused_pose;
bool success = fusion.FusePoses(fused_pose, 
                                std::chrono::milliseconds(50));

if (success) {
  std::cout << "Fused position: " 
            << fused_pose.x << ", " 
            << fused_pose.y << ", " 
            << fused_pose.z << std::endl;
}
```

### Buffer Management

```cpp
// Set maximum buffer size
fusion.SetMaxBufferSize(100);

// Get current buffer size
size_t size = fusion.GetBufferSize();

// Clear all stored localizations
fusion.Clear();
```

## JNI API (Java Interface)

### Creating and Destroying Fusion Objects

```java
// Create a pose fusion object
long fusionHandle = GpuDetectorJNI.createPoseFusion();

// Destroy when done
GpuDetectorJNI.destroyPoseFusion(fusionHandle);
```

### Adding Camera Localizations

```java
GpuDetectorJNI.addCameraLocalization(
    fusionHandle,
    cameraId,           // int
    timestampNs,        // long (nanoseconds)
    x, y, z,            // double (position)
    qw, qx, qy, qz,     // double (quaternion)
    confidence,         // double (0.0 to 1.0)
    offset_x, offset_y, offset_z,           // double (camera offset position)
    offset_qw, offset_qx, offset_qy, offset_qz  // double (camera offset orientation)
);
```

### Fusing Poses

```java
long timeWindowNs = 50_000_000;  // 50 milliseconds
double[] fusedPose = GpuDetectorJNI.fusePoses(fusionHandle, timeWindowNs);

if (fusedPose != null) {
    double x = fusedPose[0];
    double y = fusedPose[1];
    double z = fusedPose[2];
    double qw = fusedPose[3];
    double qx = fusedPose[4];
    double qy = fusedPose[5];
    double qz = fusedPose[6];
}
```

### Buffer Management

```java
// Set maximum buffer size
GpuDetectorJNI.setPoseFusionMaxBufferSize(fusionHandle, 100);

// Get current buffer size
int size = GpuDetectorJNI.getPoseFusionBufferSize(fusionHandle);

// Clear buffer
GpuDetectorJNI.clearPoseFusion(fusionHandle);
```

## Fusion Algorithm

The pose fusion uses weighted averaging with the following features:

1. **Confidence Weighting**: Each pose is weighted by its confidence factor
2. **Recency Weighting**: More recent measurements receive higher weights (exponential decay)
3. **Time Window Filtering**: Only poses within the specified time window are considered
4. **Quaternion Averaging**: Quaternions are averaged using normalized weighted sum with hemisphere checking
5. **Camera Offset Transformation**: Camera poses are transformed to the robot frame before fusion

### Weight Calculation

```
weight = confidence * exp(-age / 1_second)
```

Where:
- `confidence` is the confidence factor (0.0 to 1.0)
- `age` is the time difference from the most recent measurement in nanoseconds

### Quaternion Fusion

Quaternions are fused using weighted averaging with the following steps:
1. Ensure all quaternions are in the same hemisphere (dot product > 0)
2. Compute weighted sum of quaternion components
3. Normalize the resulting quaternion

## Example Use Case

Here's a complete example of using multiple cameras:

```cpp
frc971::apriltag::PoseFusion fusion;

// Camera 1 sees the target from the front
frc971::apriltag::CameraLocalization loc1;
loc1.camera_id = 0;
loc1.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch();
loc1.pose.x = 5.0;
loc1.pose.y = 0.0;
loc1.pose.z = 1.0;
loc1.pose.qw = 1.0;
loc1.confidence = 0.95;
loc1.camera_offset.x = 0.2;  // 20cm forward from robot center

fusion.AddCameraLocalization(loc1);

// Camera 2 sees the target from the side
frc971::apriltag::CameraLocalization loc2;
loc2.camera_id = 1;
loc2.timestamp = std::chrono::high_resolution_clock::now().time_since_epoch();
loc2.pose.x = 5.1;
loc2.pose.y = 0.1;
loc2.pose.z = 1.0;
loc2.pose.qw = 1.0;
loc2.confidence = 0.85;
loc2.camera_offset.y = 0.2;  // 20cm to the right from robot center

fusion.AddCameraLocalization(loc2);

// Fuse poses
frc971::apriltag::Pose3D fused;
if (fusion.FusePoses(fused)) {
    // Use the fused pose for robot navigation
}
```

## Performance Considerations

- The fusion algorithm is O(n) where n is the number of localizations in the time window
- Buffer trimming occurs when new localizations are added and buffer exceeds max size
- For best performance, keep the buffer size reasonable (e.g., 50-100 localizations)
- Time window should be set based on your camera update rate (typically 20-100ms)

## Notes

- All positions are in meters
- Quaternions must be unit quaternions (normalized)
- Timestamps should be monotonically increasing for best results
- Confidence values should be between 0.0 and 1.0
- Camera offsets are applied before fusion to transform poses to the robot frame
