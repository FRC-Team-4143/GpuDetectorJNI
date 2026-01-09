#include "frc971/orin/pose_fusion.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace frc971::apriltag;

// Helper function to check if two doubles are approximately equal
bool approx_equal(double a, double b, double epsilon = 1e-6) {
    return std::abs(a - b) < epsilon;
}

// Test basic pose fusion with single camera
void test_single_camera() {
    std::cout << "Test: Single camera localization" << std::endl;
    
    PoseFusion fusion;
    
    CameraLocalization loc;
    loc.camera_id = 0;
    loc.timestamp = std::chrono::nanoseconds(1000000000); // 1 second
    loc.pose.x = 1.0;
    loc.pose.y = 2.0;
    loc.pose.z = 3.0;
    loc.pose.qw = 1.0;
    loc.pose.qx = 0.0;
    loc.pose.qy = 0.0;
    loc.pose.qz = 0.0;
    loc.confidence = 1.0;
    // No camera offset
    
    fusion.AddCameraLocalization(loc);
    
    Pose3D fused;
    bool success = fusion.FusePoses(fused);
    
    assert(success);
    assert(approx_equal(fused.x, 1.0));
    assert(approx_equal(fused.y, 2.0));
    assert(approx_equal(fused.z, 3.0));
    assert(approx_equal(fused.qw, 1.0));
    
    std::cout << "  PASSED" << std::endl;
}

// Test fusion with two cameras of equal confidence
void test_two_cameras_equal_confidence() {
    std::cout << "Test: Two cameras with equal confidence" << std::endl;
    
    PoseFusion fusion;
    
    auto base_time = std::chrono::nanoseconds(1000000000);
    
    // Camera 1
    CameraLocalization loc1;
    loc1.camera_id = 0;
    loc1.timestamp = base_time;
    loc1.pose.x = 0.0;
    loc1.pose.y = 0.0;
    loc1.pose.z = 0.0;
    loc1.pose.qw = 1.0;
    loc1.pose.qx = 0.0;
    loc1.pose.qy = 0.0;
    loc1.pose.qz = 0.0;
    loc1.confidence = 1.0;
    
    // Camera 2
    CameraLocalization loc2;
    loc2.camera_id = 1;
    loc2.timestamp = base_time;
    loc2.pose.x = 2.0;
    loc2.pose.y = 2.0;
    loc2.pose.z = 2.0;
    loc2.pose.qw = 1.0;
    loc2.pose.qx = 0.0;
    loc2.pose.qy = 0.0;
    loc2.pose.qz = 0.0;
    loc2.confidence = 1.0;
    
    fusion.AddCameraLocalization(loc1);
    fusion.AddCameraLocalization(loc2);
    
    Pose3D fused;
    bool success = fusion.FusePoses(fused);
    
    assert(success);
    // Should be average of the two positions
    assert(approx_equal(fused.x, 1.0));
    assert(approx_equal(fused.y, 1.0));
    assert(approx_equal(fused.z, 1.0));
    
    std::cout << "  PASSED" << std::endl;
}

// Test fusion with different confidence levels
void test_different_confidence() {
    std::cout << "Test: Two cameras with different confidence" << std::endl;
    
    PoseFusion fusion;
    
    auto base_time = std::chrono::nanoseconds(1000000000);
    
    // Camera 1 with high confidence
    CameraLocalization loc1;
    loc1.camera_id = 0;
    loc1.timestamp = base_time;
    loc1.pose.x = 0.0;
    loc1.pose.y = 0.0;
    loc1.pose.z = 0.0;
    loc1.pose.qw = 1.0;
    loc1.pose.qx = 0.0;
    loc1.pose.qy = 0.0;
    loc1.pose.qz = 0.0;
    loc1.confidence = 0.9;
    
    // Camera 2 with low confidence
    CameraLocalization loc2;
    loc2.camera_id = 1;
    loc2.timestamp = base_time;
    loc2.pose.x = 10.0;
    loc2.pose.y = 10.0;
    loc2.pose.z = 10.0;
    loc2.pose.qw = 1.0;
    loc2.pose.qx = 0.0;
    loc2.pose.qy = 0.0;
    loc2.pose.qz = 0.0;
    loc2.confidence = 0.1;
    
    fusion.AddCameraLocalization(loc1);
    fusion.AddCameraLocalization(loc2);
    
    Pose3D fused;
    bool success = fusion.FusePoses(fused);
    
    assert(success);
    // Should be weighted more toward camera 1
    assert(fused.x < 5.0);  // Closer to 0 than 10
    assert(fused.y < 5.0);
    assert(fused.z < 5.0);
    
    std::cout << "  PASSED" << std::endl;
}

// Test time window filtering
void test_time_window() {
    std::cout << "Test: Time window filtering" << std::endl;
    
    PoseFusion fusion;
    
    auto base_time = std::chrono::nanoseconds(1000000000);
    
    // Old measurement (outside time window)
    CameraLocalization loc1;
    loc1.camera_id = 0;
    loc1.timestamp = base_time - std::chrono::milliseconds(100);
    loc1.pose.x = 10.0;
    loc1.pose.y = 10.0;
    loc1.pose.z = 10.0;
    loc1.pose.qw = 1.0;
    loc1.pose.qx = 0.0;
    loc1.pose.qy = 0.0;
    loc1.pose.qz = 0.0;
    loc1.confidence = 1.0;
    
    // Recent measurement (inside time window)
    CameraLocalization loc2;
    loc2.camera_id = 1;
    loc2.timestamp = base_time;
    loc2.pose.x = 1.0;
    loc2.pose.y = 1.0;
    loc2.pose.z = 1.0;
    loc2.pose.qw = 1.0;
    loc2.pose.qx = 0.0;
    loc2.pose.qy = 0.0;
    loc2.pose.qz = 0.0;
    loc2.confidence = 1.0;
    
    fusion.AddCameraLocalization(loc1);
    fusion.AddCameraLocalization(loc2);
    
    Pose3D fused;
    bool success = fusion.FusePoses(fused, std::chrono::milliseconds(50));
    
    assert(success);
    // Should only use loc2 since loc1 is outside time window
    assert(approx_equal(fused.x, 1.0));
    assert(approx_equal(fused.y, 1.0));
    assert(approx_equal(fused.z, 1.0));
    
    std::cout << "  PASSED" << std::endl;
}

// Test quaternion normalization
void test_quaternion_normalization() {
    std::cout << "Test: Quaternion normalization" << std::endl;
    
    Pose3D pose;
    pose.qw = 2.0;
    pose.qx = 0.0;
    pose.qy = 0.0;
    pose.qz = 0.0;
    
    pose.NormalizeQuaternion();
    
    double norm = std::sqrt(pose.qw * pose.qw + pose.qx * pose.qx + 
                           pose.qy * pose.qy + pose.qz * pose.qz);
    assert(approx_equal(norm, 1.0));
    
    std::cout << "  PASSED" << std::endl;
}

// Test buffer management
void test_buffer_management() {
    std::cout << "Test: Buffer management" << std::endl;
    
    PoseFusion fusion;
    fusion.SetMaxBufferSize(5);
    
    auto base_time = std::chrono::nanoseconds(1000000000);
    
    // Add 10 localizations
    for (int i = 0; i < 10; ++i) {
        CameraLocalization loc;
        loc.camera_id = i;
        loc.timestamp = base_time + std::chrono::milliseconds(i);
        loc.pose.x = i;
        loc.confidence = 1.0;
        fusion.AddCameraLocalization(loc);
    }
    
    // Should only keep the last 5
    assert(fusion.GetBufferSize() == 5);
    
    std::cout << "  PASSED" << std::endl;
}

int main() {
    std::cout << "Running Pose Fusion Tests..." << std::endl;
    std::cout << "================================" << std::endl;
    
    try {
        test_single_camera();
        test_two_cameras_equal_confidence();
        test_different_confidence();
        test_time_window();
        test_quaternion_normalization();
        test_buffer_management();
        
        std::cout << "================================" << std::endl;
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
