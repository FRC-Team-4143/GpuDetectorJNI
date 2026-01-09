#include "frc971/orin/pose_fusion.h"
#include <algorithm>
#include <cmath>

namespace frc971::apriltag {

PoseFusion::PoseFusion() : max_buffer_size_(100) {}

void PoseFusion::AddCameraLocalization(const CameraLocalization& localization) {
  localizations_.push_back(localization);
  TrimBuffer();
}

bool PoseFusion::FusePoses(Pose3D& fused_pose, 
                           std::chrono::nanoseconds time_window_ns) {
  if (localizations_.empty()) {
    return false;
  }
  
  // Use the most recent timestamp as reference
  auto reference_time = localizations_.back().timestamp;
  
  // Filter localizations by time window
  auto valid_localizations = FilterByTime(reference_time, time_window_ns);
  
  if (valid_localizations.empty()) {
    return false;
  }
  
  // Calculate weights for each localization
  auto weights = CalculateWeights(valid_localizations, reference_time);
  
  // Normalize weights
  double weight_sum = 0.0;
  for (double w : weights) {
    weight_sum += w;
  }
  
  if (weight_sum < 1e-9) {
    return false;
  }
  
  for (double& w : weights) {
    w /= weight_sum;
  }
  
  // Fuse positions and orientations
  FusePositions(valid_localizations, weights, fused_pose);
  FuseQuaternions(valid_localizations, weights, fused_pose);
  
  return true;
}

void PoseFusion::Clear() {
  localizations_.clear();
}

Pose3D PoseFusion::ApplyCameraOffset(const Pose3D& camera_pose, 
                                     const Pose3D& offset) const {
  Pose3D result = camera_pose;
  
  // Apply rotation from camera offset to position
  // Transform offset position by camera orientation
  double tx = offset.x;
  double ty = offset.y;
  double tz = offset.z;
  
  // Rotate offset by camera quaternion
  double qw = camera_pose.qw;
  double qx = camera_pose.qx;
  double qy = camera_pose.qy;
  double qz = camera_pose.qz;
  
  double x_rot = tx * (1 - 2*qy*qy - 2*qz*qz) + 
                 ty * (2*qx*qy - 2*qz*qw) + 
                 tz * (2*qx*qz + 2*qy*qw);
  double y_rot = tx * (2*qx*qy + 2*qz*qw) + 
                 ty * (1 - 2*qx*qx - 2*qz*qz) + 
                 tz * (2*qy*qz - 2*qx*qw);
  double z_rot = tx * (2*qx*qz - 2*qy*qw) + 
                 ty * (2*qy*qz + 2*qx*qw) + 
                 tz * (1 - 2*qx*qx - 2*qy*qy);
  
  result.x += x_rot;
  result.y += y_rot;
  result.z += z_rot;
  
  // Combine quaternions: result = camera_pose.q * offset.q
  QuaternionMultiply(camera_pose.qw, camera_pose.qx, camera_pose.qy, camera_pose.qz,
                     offset.qw, offset.qx, offset.qy, offset.qz,
                     result.qw, result.qx, result.qy, result.qz);
  
  return result;
}

void PoseFusion::FusePositions(const std::vector<CameraLocalization>& localizations,
                               const std::vector<double>& weights,
                               Pose3D& result) const {
  result.x = 0.0;
  result.y = 0.0;
  result.z = 0.0;
  
  for (size_t i = 0; i < localizations.size(); ++i) {
    // Apply camera offset to get pose in robot frame
    Pose3D transformed = ApplyCameraOffset(localizations[i].pose, 
                                           localizations[i].camera_offset);
    
    result.x += weights[i] * transformed.x;
    result.y += weights[i] * transformed.y;
    result.z += weights[i] * transformed.z;
  }
}

void PoseFusion::FuseQuaternions(const std::vector<CameraLocalization>& localizations,
                                 const std::vector<double>& weights,
                                 Pose3D& result) const {
  if (localizations.empty()) {
    result.qw = 1.0;
    result.qx = 0.0;
    result.qy = 0.0;
    result.qz = 0.0;
    return;
  }
  
  // Use weighted average quaternion method
  // Start with the first quaternion
  Pose3D transformed_first = ApplyCameraOffset(localizations[0].pose,
                                               localizations[0].camera_offset);
  
  result.qw = weights[0] * transformed_first.qw;
  result.qx = weights[0] * transformed_first.qx;
  result.qy = weights[0] * transformed_first.qy;
  result.qz = weights[0] * transformed_first.qz;
  
  // Add weighted contributions from other quaternions
  for (size_t i = 1; i < localizations.size(); ++i) {
    Pose3D transformed = ApplyCameraOffset(localizations[i].pose,
                                           localizations[i].camera_offset);
    
    // Ensure quaternions are in the same hemisphere (dot product > 0)
    double dot = result.qw * transformed.qw + 
                 result.qx * transformed.qx + 
                 result.qy * transformed.qy + 
                 result.qz * transformed.qz;
    
    double sign = (dot >= 0) ? 1.0 : -1.0;
    
    result.qw += weights[i] * sign * transformed.qw;
    result.qx += weights[i] * sign * transformed.qx;
    result.qy += weights[i] * sign * transformed.qy;
    result.qz += weights[i] * sign * transformed.qz;
  }
  
  // Normalize the resulting quaternion
  result.NormalizeQuaternion();
}

std::vector<CameraLocalization> PoseFusion::FilterByTime(
    std::chrono::nanoseconds reference_time,
    std::chrono::nanoseconds time_window) const {
  std::vector<CameraLocalization> filtered;
  
  for (const auto& loc : localizations_) {
    auto time_diff = reference_time - loc.timestamp;
    if (time_diff >= std::chrono::nanoseconds(0) && time_diff <= time_window) {
      filtered.push_back(loc);
    }
  }
  
  return filtered;
}

std::vector<double> PoseFusion::CalculateWeights(
    const std::vector<CameraLocalization>& localizations,
    std::chrono::nanoseconds reference_time) const {
  std::vector<double> weights;
  weights.reserve(localizations.size());
  
  for (const auto& loc : localizations) {
    // Weight based on confidence
    double weight = loc.confidence;
    
    // Add recency factor: more recent measurements get higher weight
    auto age = (reference_time - loc.timestamp).count();
    double recency_factor = std::exp(-age / 1e9); // decay over 1 second
    
    weight *= recency_factor;
    weights.push_back(weight);
  }
  
  return weights;
}

void PoseFusion::TrimBuffer() {
  if (localizations_.size() > max_buffer_size_) {
    // Sort by timestamp
    std::sort(localizations_.begin(), localizations_.end(),
              [](const CameraLocalization& a, const CameraLocalization& b) {
                return a.timestamp < b.timestamp;
              });
    
    // Remove oldest entries
    size_t to_remove = localizations_.size() - max_buffer_size_;
    localizations_.erase(localizations_.begin(), 
                        localizations_.begin() + to_remove);
  }
}

void PoseFusion::QuaternionMultiply(double qw1, double qx1, double qy1, double qz1,
                                    double qw2, double qx2, double qy2, double qz2,
                                    double& qw_out, double& qx_out, 
                                    double& qy_out, double& qz_out) const {
  qw_out = qw1 * qw2 - qx1 * qx2 - qy1 * qy2 - qz1 * qz2;
  qx_out = qw1 * qx2 + qx1 * qw2 + qy1 * qz2 - qz1 * qy2;
  qy_out = qw1 * qy2 - qx1 * qz2 + qy1 * qw2 + qz1 * qx2;
  qz_out = qw1 * qz2 + qx1 * qy2 - qy1 * qx2 + qz1 * qw2;
}

}  // namespace frc971::apriltag
