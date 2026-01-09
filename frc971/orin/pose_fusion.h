#pragma once

#include <chrono>
#include <vector>
#include <cmath>

namespace frc971::apriltag {

// Represents a 3D pose with position and quaternion orientation
struct Pose3D {
  // Position in meters
  double x;
  double y;
  double z;
  
  // Orientation as unit quaternion (w, x, y, z)
  double qw;
  double qx;
  double qy;
  double qz;
  
  Pose3D() : x(0), y(0), z(0), qw(1), qx(0), qy(0), qz(0) {}
  
  Pose3D(double x_, double y_, double z_, double qw_, double qx_, double qy_, double qz_)
    : x(x_), y(y_), z(z_), qw(qw_), qx(qx_), qy(qy_), qz(qz_) {}
  
  // Normalize the quaternion
  void NormalizeQuaternion() {
    double norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 1e-9) {
      qw /= norm;
      qx /= norm;
      qy /= norm;
      qz /= norm;
    }
  }
};

// Represents a camera's localization with associated metadata
struct CameraLocalization {
  // The pose estimate from this camera
  Pose3D pose;
  
  // Confidence factor (0.0 to 1.0), higher is more confident
  double confidence;
  
  // Timestamp in nanoseconds since epoch
  std::chrono::nanoseconds timestamp;
  
  // Camera offset relative to robot center
  Pose3D camera_offset;
  
  // Camera identifier
  int camera_id;
  
  CameraLocalization()
    : confidence(0.0),
      timestamp(0),
      camera_id(-1) {}
};

// Multi-camera pose fusion using weighted averaging
class PoseFusion {
 public:
  PoseFusion();
  ~PoseFusion() = default;
  
  // Add a camera localization to the fusion buffer
  void AddCameraLocalization(const CameraLocalization& localization);
  
  // Fuse all camera localizations within the time window
  // Returns true if fusion was successful, false if no valid data
  bool FusePoses(Pose3D& fused_pose, 
                 std::chrono::nanoseconds time_window_ns = std::chrono::milliseconds(50));
  
  // Clear all stored localizations
  void Clear();
  
  // Set the maximum number of localizations to keep in buffer
  void SetMaxBufferSize(size_t max_size) { max_buffer_size_ = max_size; }
  
  // Get the number of localizations currently in buffer
  size_t GetBufferSize() const { return localizations_.size(); }
  
 private:
  // Apply camera offset to transform pose from camera frame to robot frame
  Pose3D ApplyCameraOffset(const Pose3D& camera_pose, const Pose3D& offset) const;
  
  // Weighted average of positions
  void FusePositions(const std::vector<CameraLocalization>& localizations,
                     const std::vector<double>& weights,
                     Pose3D& result) const;
  
  // Weighted average of quaternions using normalized quaternion interpolation
  void FuseQuaternions(const std::vector<CameraLocalization>& localizations,
                       const std::vector<double>& weights,
                       Pose3D& result) const;
  
  // Filter localizations by timestamp
  std::vector<CameraLocalization> FilterByTime(
      std::chrono::nanoseconds reference_time,
      std::chrono::nanoseconds time_window) const;
  
  // Calculate weights based on confidence and recency
  std::vector<double> CalculateWeights(
      const std::vector<CameraLocalization>& localizations,
      std::chrono::nanoseconds reference_time) const;
  
  // Remove old localizations to maintain buffer size
  void TrimBuffer();
  
  // Multiply two quaternions
  void QuaternionMultiply(double qw1, double qx1, double qy1, double qz1,
                          double qw2, double qx2, double qy2, double qz2,
                          double& qw_out, double& qx_out, 
                          double& qy_out, double& qz_out) const;
  
  // Storage for camera localizations
  std::vector<CameraLocalization> localizations_;
  
  // Maximum number of localizations to keep
  size_t max_buffer_size_;
};

}  // namespace frc971::apriltag
