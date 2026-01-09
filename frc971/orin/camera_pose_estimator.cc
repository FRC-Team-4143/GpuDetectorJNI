#include "frc971/orin/camera_pose_estimator.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#include <iostream>

namespace frc971::apriltag {

CameraPoseEstimator::CameraPoseEstimator(const CameraMatrix& camera_matrix,
                                         const DistCoeffs& dist_coeffs)
    : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs) {}

void CameraPoseEstimator::AddTagToMap(const TagPose& tag) {
  tag_map_[tag.tag_id] = tag;
}

void CameraPoseEstimator::RemoveTagFromMap(int tag_id) {
  tag_map_.erase(tag_id);
}

void CameraPoseEstimator::ClearTagMap() {
  tag_map_.clear();
}

cv::Mat CameraPoseEstimator::QuaternionToRotationMatrix(double qw, double qx, 
                                                        double qy, double qz) const {
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  
  // Normalize quaternion
  double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (norm < 1e-9) {
    return R;  // Return identity
  }
  qw /= norm; qx /= norm; qy /= norm; qz /= norm;
  
  // Convert to rotation matrix
  R.at<double>(0, 0) = 1 - 2*qy*qy - 2*qz*qz;
  R.at<double>(0, 1) = 2*qx*qy - 2*qz*qw;
  R.at<double>(0, 2) = 2*qx*qz + 2*qy*qw;
  
  R.at<double>(1, 0) = 2*qx*qy + 2*qz*qw;
  R.at<double>(1, 1) = 1 - 2*qx*qx - 2*qz*qz;
  R.at<double>(1, 2) = 2*qy*qz - 2*qx*qw;
  
  R.at<double>(2, 0) = 2*qx*qz - 2*qy*qw;
  R.at<double>(2, 1) = 2*qy*qz + 2*qx*qw;
  R.at<double>(2, 2) = 1 - 2*qx*qx - 2*qy*qy;
  
  return R;
}

void CameraPoseEstimator::RotationMatrixToQuaternion(const cv::Mat& R,
                                                     double& qw, double& qx, 
                                                     double& qy, double& qz) const {
  double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
  
  if (trace > 0) {
    double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
    qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
    qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
  } else if (R.at<double>(0, 0) > R.at<double>(1, 1) && 
             R.at<double>(0, 0) > R.at<double>(2, 2)) {
    double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) - 
                                R.at<double>(1, 1) - R.at<double>(2, 2));
    qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
    qx = 0.25 * s;
    qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
    qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
  } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
    double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) - 
                                R.at<double>(0, 0) - R.at<double>(2, 2));
    qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
    qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
    qy = 0.25 * s;
    qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
  } else {
    double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) - 
                                R.at<double>(0, 0) - R.at<double>(1, 1));
    qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
    qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
    qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
    qz = 0.25 * s;
  }
}

std::vector<cv::Point3d> CameraPoseEstimator::GetTagWorldCorners(
    const TagPose& tag) const {
  std::vector<cv::Point3d> corners;
  
  // Tag corners in tag frame (assuming tag lies in XY plane, Z=0)
  // Corners are ordered counter-clockwise from bottom-left
  double half_size = tag.size / 2.0;
  std::vector<cv::Point3d> tag_frame_corners = {
    cv::Point3d(-half_size, -half_size, 0),  // bottom-left
    cv::Point3d( half_size, -half_size, 0),  // bottom-right
    cv::Point3d( half_size,  half_size, 0),  // top-right
    cv::Point3d(-half_size,  half_size, 0)   // top-left
  };
  
  // Convert tag orientation to rotation matrix
  cv::Mat R_tag = QuaternionToRotationMatrix(tag.qw, tag.qx, tag.qy, tag.qz);
  cv::Mat t_tag = (cv::Mat_<double>(3, 1) << tag.x, tag.y, tag.z);
  
  // Transform corners to world frame
  for (const auto& corner : tag_frame_corners) {
    cv::Mat corner_tag = (cv::Mat_<double>(3, 1) << corner.x, corner.y, corner.z);
    cv::Mat corner_world = R_tag * corner_tag + t_tag;
    corners.emplace_back(corner_world.at<double>(0), 
                        corner_world.at<double>(1), 
                        corner_world.at<double>(2));
  }
  
  return corners;
}

cv::Mat CameraPoseEstimator::GetCameraMatrixCV() const {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = camera_matrix_.fx;
  K.at<double>(0, 2) = camera_matrix_.cx;
  K.at<double>(1, 1) = camera_matrix_.fy;
  K.at<double>(1, 2) = camera_matrix_.cy;
  return K;
}

cv::Mat CameraPoseEstimator::GetDistCoeffsCV() const {
  cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);
  dist.at<double>(0) = dist_coeffs_.k1;
  dist.at<double>(1) = dist_coeffs_.k2;
  dist.at<double>(2) = dist_coeffs_.p1;
  dist.at<double>(3) = dist_coeffs_.p2;
  dist.at<double>(4) = dist_coeffs_.k3;
  return dist;
}

bool CameraPoseEstimator::EstimateCameraPose(const zarray_t* detections,
                                             CameraPose& camera_pose,
                                             CameraPose* initial_estimate) {
  if (!detections || zarray_size(detections) == 0) {
    return false;
  }
  
  // Collect 3D-2D correspondences from all visible tags
  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;
  
  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    
    // Check if this tag is in our map
    auto it = tag_map_.find(det->id);
    if (it == tag_map_.end()) {
      continue;  // Skip unknown tags
    }
    
    const TagPose& tag = it->second;
    
    // Get 3D corners in world frame
    std::vector<cv::Point3d> world_corners = GetTagWorldCorners(tag);
    
    // Add all 4 corners as correspondences
    for (int j = 0; j < 4; ++j) {
      object_points.push_back(world_corners[j]);
      image_points.emplace_back(det->p[j][0], det->p[j][1]);
    }
  }
  
  if (object_points.size() < 4) {
    // Need at least 4 points (1 tag) for PnP
    return false;
  }
  
  // Prepare camera parameters
  cv::Mat camera_matrix_cv = GetCameraMatrixCV();
  cv::Mat dist_coeffs_cv = GetDistCoeffsCV();
  
  // Solve PnP
  cv::Mat rvec, tvec;
  bool use_extrinsic_guess = false;
  
  if (initial_estimate) {
    // Use initial estimate if provided
    cv::Mat R_init = QuaternionToRotationMatrix(initial_estimate->qw,
                                                initial_estimate->qx,
                                                initial_estimate->qy,
                                                initial_estimate->qz);
    cv::Rodrigues(R_init, rvec);
    tvec = (cv::Mat_<double>(3, 1) << initial_estimate->x, 
                                       initial_estimate->y, 
                                       initial_estimate->z);
    use_extrinsic_guess = true;
  }
  
  bool success;
  if (object_points.size() >= 6) {
    // Use iterative method for better accuracy with more points
    success = cv::solvePnP(object_points, image_points, 
                          camera_matrix_cv, dist_coeffs_cv,
                          rvec, tvec, use_extrinsic_guess, 
                          cv::SOLVEPNP_ITERATIVE);
  } else {
    // Use P3P for minimal case
    success = cv::solvePnP(object_points, image_points,
                          camera_matrix_cv, dist_coeffs_cv,
                          rvec, tvec, use_extrinsic_guess,
                          cv::SOLVEPNP_P3P);
  }
  
  if (!success) {
    return false;
  }
  
  // Convert result to camera pose
  // Note: solvePnP gives camera pose in world frame
  cv::Mat R_cam;
  cv::Rodrigues(rvec, R_cam);
  
  // Extract position
  camera_pose.x = tvec.at<double>(0);
  camera_pose.y = tvec.at<double>(1);
  camera_pose.z = tvec.at<double>(2);
  
  // Convert rotation to quaternion
  RotationMatrixToQuaternion(R_cam, camera_pose.qw, camera_pose.qx, 
                            camera_pose.qy, camera_pose.qz);
  
  // Calculate reprojection error
  std::vector<cv::Point2d> projected_points;
  cv::projectPoints(object_points, rvec, tvec, 
                   camera_matrix_cv, dist_coeffs_cv, projected_points);
  
  double error_sum = 0.0;
  for (size_t i = 0; i < image_points.size(); ++i) {
    double dx = image_points[i].x - projected_points[i].x;
    double dy = image_points[i].y - projected_points[i].y;
    error_sum += std::sqrt(dx*dx + dy*dy);
  }
  camera_pose.error = error_sum / image_points.size();
  
  return true;
}

bool CameraPoseEstimator::EstimateCameraPoseFromSingleTag(
    const apriltag_detection_t* detection,
    CameraPose& camera_pose) {
  if (!detection) {
    return false;
  }
  
  // Check if this tag is in our map
  auto it = tag_map_.find(detection->id);
  if (it == tag_map_.end()) {
    return false;
  }
  
  const TagPose& tag = it->second;
  
  // Get 3D corners in world frame
  std::vector<cv::Point3d> object_points = GetTagWorldCorners(tag);
  
  // Get 2D corners from detection
  std::vector<cv::Point2d> image_points;
  for (int i = 0; i < 4; ++i) {
    image_points.emplace_back(detection->p[i][0], detection->p[i][1]);
  }
  
  // Prepare camera parameters
  cv::Mat camera_matrix_cv = GetCameraMatrixCV();
  cv::Mat dist_coeffs_cv = GetDistCoeffsCV();
  
  // Solve PnP
  cv::Mat rvec, tvec;
  bool success = cv::solvePnP(object_points, image_points,
                              camera_matrix_cv, dist_coeffs_cv,
                              rvec, tvec, false, cv::SOLVEPNP_IPPE);
  
  if (!success) {
    return false;
  }
  
  // Convert result to camera pose
  cv::Mat R_cam;
  cv::Rodrigues(rvec, R_cam);
  
  camera_pose.x = tvec.at<double>(0);
  camera_pose.y = tvec.at<double>(1);
  camera_pose.z = tvec.at<double>(2);
  
  RotationMatrixToQuaternion(R_cam, camera_pose.qw, camera_pose.qx,
                            camera_pose.qy, camera_pose.qz);
  
  // Calculate reprojection error
  std::vector<cv::Point2d> projected_points;
  cv::projectPoints(object_points, rvec, tvec,
                   camera_matrix_cv, dist_coeffs_cv, projected_points);
  
  double error_sum = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    double dx = image_points[i].x - projected_points[i].x;
    double dy = image_points[i].y - projected_points[i].y;
    error_sum += std::sqrt(dx*dx + dy*dy);
  }
  camera_pose.error = error_sum / 4.0;
  
  return true;
}

}  // namespace frc971::apriltag
