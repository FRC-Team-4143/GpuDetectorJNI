#pragma once

#include <opencv2/core.hpp>
#include <map>
#include <vector>
#include "third_party/apriltag/apriltag.h"
#include "frc971/orin/971apriltag.h"

namespace frc971::apriltag {

// Represents a known AprilTag's position and orientation in the world frame
struct TagPose {
  int tag_id;
  
  // Position in meters (world frame)
  double x;
  double y;
  double z;
  
  // Orientation as quaternion (world frame)
  double qw;
  double qx;
  double qy;
  double qz;
  
  // Tag size in meters
  double size;
  
  TagPose() : tag_id(-1), x(0), y(0), z(0), 
              qw(1), qx(0), qy(0), qz(0), size(0.1524) {}
  
  TagPose(int id, double x_, double y_, double z_, 
          double qw_, double qx_, double qy_, double qz_, double size_)
    : tag_id(id), x(x_), y(y_), z(z_), 
      qw(qw_), qx(qx_), qy(qy_), qz(qz_), size(size_) {}
};

// Camera pose in world frame
struct CameraPose {
  // Position in meters
  double x;
  double y;
  double z;
  
  // Orientation as quaternion
  double qw;
  double qx;
  double qy;
  double qz;
  
  // Reprojection error
  double error;
  
  CameraPose() : x(0), y(0), z(0), qw(1), qx(0), qy(0), qz(0), error(0) {}
};

// Estimates camera pose using Perspective-n-Point algorithm
class CameraPoseEstimator {
 public:
  CameraPoseEstimator(const CameraMatrix& camera_matrix, 
                      const DistCoeffs& dist_coeffs);
  ~CameraPoseEstimator() = default;
  
  // Add a known tag to the map
  void AddTagToMap(const TagPose& tag);
  
  // Remove a tag from the map
  void RemoveTagFromMap(int tag_id);
  
  // Clear all tags from the map
  void ClearTagMap();
  
  // Get number of tags in map
  size_t GetTagMapSize() const { return tag_map_.size(); }
  
  // Estimate camera pose from detected tags
  // Returns true if successful, false if insufficient tags or error
  bool EstimateCameraPose(const zarray_t* detections, 
                          CameraPose& camera_pose,
                          CameraPose* initial_estimate = nullptr);
  
  // Estimate camera pose from a single detection (for debugging/testing)
  bool EstimateCameraPoseFromSingleTag(const apriltag_detection_t* detection,
                                       CameraPose& camera_pose);
  
  // Update camera parameters
  void SetCameraMatrix(const CameraMatrix& camera_matrix) { 
    camera_matrix_ = camera_matrix; 
  }
  
  void SetDistortionCoeffs(const DistCoeffs& dist_coeffs) { 
    dist_coeffs_ = dist_coeffs; 
  }
  
 private:
  // Convert quaternion to rotation matrix (OpenCV format)
  cv::Mat QuaternionToRotationMatrix(double qw, double qx, double qy, double qz) const;
  
  // Convert rotation matrix to quaternion
  void RotationMatrixToQuaternion(const cv::Mat& R, 
                                  double& qw, double& qx, double& qy, double& qz) const;
  
  // Get tag's 3D corner positions in world frame
  std::vector<cv::Point3d> GetTagWorldCorners(const TagPose& tag) const;
  
  // Convert camera matrix to OpenCV format
  cv::Mat GetCameraMatrixCV() const;
  
  // Convert distortion coefficients to OpenCV format
  cv::Mat GetDistCoeffsCV() const;
  
  // Map of tag ID to tag pose in world frame
  std::map<int, TagPose> tag_map_;
  
  // Camera intrinsic parameters
  CameraMatrix camera_matrix_;
  DistCoeffs dist_coeffs_;
};

}  // namespace frc971::apriltag
