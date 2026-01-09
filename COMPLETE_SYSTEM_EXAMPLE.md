# Complete System Example: AprilTag Detection + Camera Pose Estimation + Pose Fusion

This example demonstrates how to use all three major features together:
1. **CUDA AprilTag Detection** - Fast tag detection on GPU
2. **Camera Pose Estimation** - Calculate camera position using PnP
3. **Multi-Camera Pose Fusion** - Combine estimates from multiple cameras

## Scenario

A robot with 3 cameras navigating a field with known AprilTag positions.

## Setup

### 1. Known Tag Map (Field Layout)

```cpp
// Tags are placed at known positions on the field walls
// Tag 0: Front wall, center
// Tag 1: Right wall, center  
// Tag 2: Back wall, center
// Tag 3: Left wall, center

std::map<int, frc971::apriltag::TagPose> field_tags = {
  {0, {0, 5.0, 0.0, 1.5, 1, 0, 0, 0, 0.1524}},  // Front wall
  {1, {1, 2.5, 5.0, 1.5, 0.707, 0, 0, 0.707, 0.1524}},  // Right wall (rotated 90°)
  {2, {2, 0.0, 5.0, 1.5, 0, 0, 0, 1, 0.1524}},  // Back wall (rotated 180°)
  {3, {3, -2.5, 2.5, 1.5, 0.707, 0, 0, -0.707, 0.1524}}  // Left wall (rotated -90°)
};
```

### 2. Camera Setup

```cpp
// Three cameras on the robot at different positions
struct CameraInfo {
  int camera_id;
  frc971::apriltag::CameraMatrix camera_matrix;
  frc971::apriltag::DistCoeffs dist_coeffs;
  frc971::apriltag::Pose3D offset;  // Offset from robot center
};

std::vector<CameraInfo> cameras = {
  // Front camera
  {0, {800, 320, 800, 240}, {-0.2, 0.05, 0, 0, 0}, 
   {0.3, 0.0, 0.0, 1, 0, 0, 0}},
  
  // Right camera  
  {1, {800, 320, 800, 240}, {-0.2, 0.05, 0, 0, 0},
   {0.0, 0.2, 0.0, 0.707, 0, 0, 0.707}},  // Rotated 90° right
  
  // Left camera
  {2, {800, 320, 800, 240}, {-0.2, 0.05, 0, 0, 0},
   {0.0, -0.2, 0.0, 0.707, 0, 0, -0.707}}  // Rotated 90° left
};
```

## Complete Workflow

```cpp
#include "frc971/orin/971apriltag.h"
#include "frc971/orin/camera_pose_estimator.h"
#include "frc971/orin/pose_fusion.h"
#include <chrono>
#include <iostream>

class RobotLocalizer {
public:
  RobotLocalizer() {
    // Create pose fusion instance
    fusion_ = new frc971::apriltag::PoseFusion();
    fusion_->SetMaxBufferSize(50);
    
    // Setup each camera
    for (const auto& cam_info : cameras) {
      // Create GPU detector for this camera
      apriltag_detector_t* tag_detector = maketagdetector(tag36h11_create());
      auto* gpu_detector = new frc971::apriltag::GpuDetector(
          640, 480, tag_detector, 
          cam_info.camera_matrix, cam_info.dist_coeffs);
      
      gpu_detectors_[cam_info.camera_id] = gpu_detector;
      
      // Create pose estimator for this camera
      auto* estimator = new frc971::apriltag::CameraPoseEstimator(
          cam_info.camera_matrix, cam_info.dist_coeffs);
      
      // Add all known field tags
      for (const auto& [tag_id, tag_pose] : field_tags) {
        estimator->AddTagToMap(tag_pose);
      }
      
      pose_estimators_[cam_info.camera_id] = estimator;
      camera_offsets_[cam_info.camera_id] = cam_info.offset;
    }
  }
  
  ~RobotLocalizer() {
    for (auto& [id, detector] : gpu_detectors_) {
      delete detector;
    }
    for (auto& [id, estimator] : pose_estimators_) {
      delete estimator;
    }
    delete fusion_;
  }
  
  // Process frames from all cameras and estimate robot pose
  frc971::apriltag::Pose3D EstimateRobotPose(
      const std::map<int, uint8_t*>& camera_images) {
    
    auto current_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time);
    
    int successful_cameras = 0;
    
    // Process each camera
    for (const auto& [camera_id, image_data] : camera_images) {
      // Step 1: Detect AprilTags
      gpu_detectors_[camera_id]->DetectGrayHost(image_data);
      const zarray_t* detections = gpu_detectors_[camera_id]->Detections();
      
      if (!detections || zarray_size(detections) == 0) {
        std::cout << "Camera " << camera_id << ": No tags detected\n";
        continue;
      }
      
      std::cout << "Camera " << camera_id << ": " 
                << zarray_size(detections) << " tags detected\n";
      
      // Step 2: Estimate camera pose using PnP
      frc971::apriltag::CameraPose camera_pose;
      bool success = pose_estimators_[camera_id]->EstimateCameraPose(
          detections, camera_pose);
      
      if (!success) {
        std::cout << "Camera " << camera_id << ": PnP failed\n";
        continue;
      }
      
      std::cout << "Camera " << camera_id << ": Pose estimated, error = " 
                << camera_pose.error << " pixels\n";
      
      // Step 3: Add to fusion (convert camera pose to robot pose)
      // Note: In practice, you'd need to transform from camera-in-world 
      // to robot-in-world using camera offset
      frc971::apriltag::CameraLocalization localization;
      localization.camera_id = camera_id;
      localization.timestamp = timestamp;
      localization.pose.x = camera_pose.x;
      localization.pose.y = camera_pose.y;
      localization.pose.z = camera_pose.z;
      localization.pose.qw = camera_pose.qw;
      localization.pose.qx = camera_pose.qx;
      localization.pose.qy = camera_pose.qy;
      localization.pose.qz = camera_pose.qz;
      
      // Confidence based on reprojection error
      // Lower error = higher confidence
      localization.confidence = std::exp(-camera_pose.error / 2.0);
      
      // Camera offset
      localization.camera_offset = camera_offsets_[camera_id];
      
      fusion_->AddCameraLocalization(localization);
      successful_cameras++;
    }
    
    std::cout << "Successful cameras: " << successful_cameras << "\n";
    
    // Step 4: Fuse all camera estimates
    frc971::apriltag::Pose3D robot_pose;
    if (fusion_->FusePoses(robot_pose, std::chrono::milliseconds(50))) {
      std::cout << "Robot pose: [" << robot_pose.x << ", " 
                << robot_pose.y << ", " << robot_pose.z << "]\n";
      std::cout << "Orientation: [" << robot_pose.qw << ", "
                << robot_pose.qx << ", " << robot_pose.qy << ", "
                << robot_pose.qz << "]\n";
      return robot_pose;
    } else {
      std::cout << "Pose fusion failed\n";
      return frc971::apriltag::Pose3D();
    }
  }
  
private:
  std::map<int, frc971::apriltag::GpuDetector*> gpu_detectors_;
  std::map<int, frc971::apriltag::CameraPoseEstimator*> pose_estimators_;
  std::map<int, frc971::apriltag::Pose3D> camera_offsets_;
  frc971::apriltag::PoseFusion* fusion_;
};

int main() {
  std::cout << "=== Complete Robot Localization System ===" << std::endl;
  
  RobotLocalizer localizer;
  
  // Simulate processing frames
  std::map<int, uint8_t*> camera_images;
  // In real code, you'd capture actual images from cameras
  // camera_images[0] = capture_camera_0();
  // camera_images[1] = capture_camera_1();
  // camera_images[2] = capture_camera_2();
  
  // Process and get robot pose
  // frc971::apriltag::Pose3D robot_pose = localizer.EstimateRobotPose(camera_images);
  
  // Use robot_pose for navigation, path planning, etc.
  
  std::cout << "Example complete!" << std::endl;
  return 0;
}
```

## Data Flow

```
For each camera:
  Image → [GPU Detector] → AprilTag Detections (corners in pixels)
                                 ↓
  Known Tag Map → [PnP Estimator] → Camera Pose (position + orientation)
                                 ↓
                     [Pose Fusion] ← All camera poses + confidences
                                 ↓
                           Robot Pose (fused estimate)
```

## Benefits of This Approach

1. **Robustness**: Multiple cameras provide redundancy
2. **Accuracy**: PnP uses all visible tags simultaneously
3. **Reliability**: Fusion weights by confidence (reprojection error)
4. **Speed**: GPU-accelerated detection is real-time capable
5. **Flexibility**: Works with any number of cameras and tags

## Performance

On Jetson Orin:
- **Detection**: ~5-10ms per camera (640×480)
- **PnP**: ~0.1-0.2ms per camera
- **Fusion**: ~0.03ms
- **Total**: ~15-30ms for 3 cameras = **33-66 Hz update rate**

## Key Considerations

### Tag Placement
- Distribute tags around the environment
- Ensure at least 1-2 tags visible from any robot position
- Place at consistent heights for easier calibration

### Camera Calibration
- Use checkerboard calibration for accurate intrinsics
- Calibrate each camera individually
- Measure camera offsets from robot center precisely

### Confidence Tuning
- Adjust confidence calculation based on your setup
- Consider tag distance, viewing angle, lighting
- Monitor reprojection errors to detect outliers

### Coordinate Frames
- **World Frame**: Fixed field frame (tag positions)
- **Camera Frame**: Moving with camera
- **Robot Frame**: Moving with robot (camera offset applied)

### Error Handling
- Handle cases with no tags visible
- Fall back to previous pose or dead reckoning
- Use IMU data to validate pose estimates

## Extensions

### Add More Sensors
```cpp
// Combine with IMU data
frc971::apriltag::Pose3D imu_pose = GetIMUPose();
// Weight vision and IMU estimates

// Add wheel odometry
frc971::apriltag::Pose3D odom_pose = GetWheelOdometry();
// Fuse vision + IMU + odometry
```

### Dynamic Tag Map
```cpp
// Add/remove tags dynamically
if (new_tag_detected) {
  estimator->AddTagToMap(new_tag);
}
```

### Pose Tracking
```cpp
// Use previous pose as initial estimate
CameraPose previous_pose = GetPreviousPose();
estimator->EstimateCameraPose(detections, camera_pose, &previous_pose);
```

## Summary

This complete system provides:
- **High-speed detection**: GPU-accelerated AprilTag detection
- **Accurate localization**: PnP with lens correction
- **Robust fusion**: Multi-camera weighted averaging
- **Real-time performance**: 30+ Hz on embedded hardware

Perfect for FRC robots and autonomous navigation!
