#include "frc971/orin/camera_pose_estimator.h"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/common/zarray.h"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>

using namespace frc971::apriltag;

// Helper to check approximate equality
bool approx_equal(double a, double b, double epsilon = 0.01) {
    return std::abs(a - b) < epsilon;
}

// Helper to print camera pose
void print_pose(const CameraPose& pose, const std::string& label) {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << label << ":\n";
    std::cout << "  Position: [" << pose.x << ", " << pose.y << ", " << pose.z << "]\n";
    std::cout << "  Quaternion: [" << pose.qw << ", " << pose.qx << ", " 
              << pose.qy << ", " << pose.qz << "]\n";
    std::cout << "  Error: " << pose.error << " pixels\n";
}

// Create a synthetic detection
apriltag_detection_t* create_synthetic_detection(int id, const double corners[4][2]) {
    apriltag_detection_t* det = new apriltag_detection_t();
    det->id = id;
    for (int i = 0; i < 4; ++i) {
        det->p[i][0] = corners[i][0];
        det->p[i][1] = corners[i][1];
    }
    return det;
}

void test_single_tag_frontal() {
    std::cout << "Test: Single tag frontal view" << std::endl;
    
    // Setup camera parameters (typical webcam)
    CameraMatrix camera_matrix{800, 320, 800, 240};  // fx, cx, fy, cy
    DistCoeffs dist_coeffs{0, 0, 0, 0, 0};  // No distortion
    
    CameraPoseEstimator estimator(camera_matrix, dist_coeffs);
    
    // Add a tag at origin, facing +Z direction, 6 inches (0.1524m) size
    TagPose tag(0, 0, 0, 0, 1, 0, 0, 0, 0.1524);
    estimator.AddTagToMap(tag);
    
    // Camera is at Z=1m, looking at origin
    // Expected detection corners (approx, assuming camera at (0, 0, 1))
    // Tag corners project to image based on camera matrix
    double corners[4][2] = {
        {259.0, 301.0},  // bottom-left
        {381.0, 301.0},  // bottom-right
        {381.0, 179.0},  // top-right
        {259.0, 179.0}   // top-left
    };
    
    apriltag_detection_t* det = create_synthetic_detection(0, corners);
    
    CameraPose camera_pose;
    bool success = estimator.EstimateCameraPoseFromSingleTag(det, camera_pose);
    
    delete det;
    
    assert(success);
    print_pose(camera_pose, "Estimated Camera Pose");
    
    // Camera should be approximately at (0, 0, 1)
    std::cout << "  Expected: ~[0, 0, 1]" << std::endl;
    
    // Check Z is positive (camera in front of tag)
    assert(camera_pose.z > 0.5);
    
    std::cout << "  PASSED\n" << std::endl;
}

void test_tag_map_management() {
    std::cout << "Test: Tag map management" << std::endl;
    
    CameraMatrix camera_matrix{800, 320, 800, 240};
    DistCoeffs dist_coeffs{0, 0, 0, 0, 0};
    
    CameraPoseEstimator estimator(camera_matrix, dist_coeffs);
    
    // Add multiple tags
    estimator.AddTagToMap(TagPose(0, 0, 0, 0, 1, 0, 0, 0, 0.1524));
    estimator.AddTagToMap(TagPose(1, 1, 0, 0, 1, 0, 0, 0, 0.1524));
    estimator.AddTagToMap(TagPose(2, 0, 1, 0, 1, 0, 0, 0, 0.1524));
    
    assert(estimator.GetTagMapSize() == 3);
    std::cout << "  Added 3 tags" << std::endl;
    
    // Remove one tag
    estimator.RemoveTagFromMap(1);
    assert(estimator.GetTagMapSize() == 2);
    std::cout << "  Removed 1 tag, now 2 remain" << std::endl;
    
    // Clear all
    estimator.ClearTagMap();
    assert(estimator.GetTagMapSize() == 0);
    std::cout << "  Cleared all tags" << std::endl;
    
    std::cout << "  PASSED\n" << std::endl;
}

void test_multiple_tags() {
    std::cout << "Test: Multiple tags PnP" << std::endl;
    
    CameraMatrix camera_matrix{800, 320, 800, 240};
    DistCoeffs dist_coeffs{0, 0, 0, 0, 0};
    
    CameraPoseEstimator estimator(camera_matrix, dist_coeffs);
    
    // Add two tags in known positions
    estimator.AddTagToMap(TagPose(0, 0, 0, 0, 1, 0, 0, 0, 0.1524));
    estimator.AddTagToMap(TagPose(1, 0.5, 0, 0, 1, 0, 0, 0, 0.1524));
    
    // Create synthetic detections for both tags
    zarray_t* detections = zarray_create(sizeof(apriltag_detection_t*));
    
    double corners0[4][2] = {
        {259.0, 301.0},
        {381.0, 301.0},
        {381.0, 179.0},
        {259.0, 179.0}
    };
    
    double corners1[4][2] = {
        {459.0, 301.0},
        {581.0, 301.0},
        {581.0, 179.0},
        {459.0, 179.0}
    };
    
    apriltag_detection_t* det0 = create_synthetic_detection(0, corners0);
    apriltag_detection_t* det1 = create_synthetic_detection(1, corners1);
    
    zarray_add(detections, &det0);
    zarray_add(detections, &det1);
    
    CameraPose camera_pose;
    bool success = estimator.EstimateCameraPose(detections, camera_pose);
    
    delete det0;
    delete det1;
    zarray_destroy(detections);
    
    assert(success);
    print_pose(camera_pose, "Estimated Camera Pose (2 tags)");
    
    std::cout << "  Using 2 tags should give better estimate" << std::endl;
    std::cout << "  PASSED\n" << std::endl;
}

void test_camera_parameter_update() {
    std::cout << "Test: Update camera parameters" << std::endl;
    
    CameraMatrix camera_matrix1{800, 320, 800, 240};
    DistCoeffs dist_coeffs1{0, 0, 0, 0, 0};
    
    CameraPoseEstimator estimator(camera_matrix1, dist_coeffs1);
    
    // Update parameters
    CameraMatrix camera_matrix2{1000, 400, 1000, 300};
    DistCoeffs dist_coeffs2{0.1, 0.01, 0, 0, 0};
    
    estimator.SetCameraMatrix(camera_matrix2);
    estimator.SetDistortionCoeffs(dist_coeffs2);
    
    std::cout << "  Updated camera parameters successfully" << std::endl;
    std::cout << "  PASSED\n" << std::endl;
}

void test_unknown_tag() {
    std::cout << "Test: Detection with unknown tag ID" << std::endl;
    
    CameraMatrix camera_matrix{800, 320, 800, 240};
    DistCoeffs dist_coeffs{0, 0, 0, 0, 0};
    
    CameraPoseEstimator estimator(camera_matrix, dist_coeffs);
    
    // Add tag 0
    estimator.AddTagToMap(TagPose(0, 0, 0, 0, 1, 0, 0, 0, 0.1524));
    
    // Try to estimate with tag 99 (not in map)
    double corners[4][2] = {
        {259.0, 301.0},
        {381.0, 301.0},
        {381.0, 179.0},
        {259.0, 179.0}
    };
    
    apriltag_detection_t* det = create_synthetic_detection(99, corners);
    
    CameraPose camera_pose;
    bool success = estimator.EstimateCameraPoseFromSingleTag(det, camera_pose);
    
    delete det;
    
    assert(!success);
    std::cout << "  Correctly rejected unknown tag" << std::endl;
    std::cout << "  PASSED\n" << std::endl;
}

int main() {
    std::cout << "=== Camera Pose Estimator Tests ===" << std::endl;
    std::cout << std::endl;
    
    try {
        test_single_tag_frontal();
        test_tag_map_management();
        test_multiple_tags();
        test_camera_parameter_update();
        test_unknown_tag();
        
        std::cout << "===================================" << std::endl;
        std::cout << "All tests PASSED!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
}
