#include "frc971/orin/pose_fusion.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace frc971::apriltag;

void printPose(const Pose3D& pose, const std::string& label) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << label << ":\n";
    std::cout << "  Position: [" << pose.x << ", " << pose.y << ", " << pose.z << "]\n";
    std::cout << "  Quaternion: [" << pose.qw << ", " << pose.qx << ", " 
              << pose.qy << ", " << pose.qz << "]\n";
}

int main() {
    std::cout << "=== Multi-Camera Pose Fusion Demo ===\n\n";
    
    // Create a pose fusion object
    PoseFusion fusion;
    fusion.SetMaxBufferSize(50);
    
    std::cout << "Scenario: Robot with 3 cameras tracking an AprilTag\n\n";
    
    // Simulate 3 cameras with different positions on the robot
    auto current_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    
    // Front camera (high confidence, sees target head-on)
    CameraLocalization front_camera;
    front_camera.camera_id = 0;
    front_camera.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time);
    front_camera.pose.x = 3.0;
    front_camera.pose.y = 0.1;
    front_camera.pose.z = 1.5;
    front_camera.pose.qw = 1.0;
    front_camera.pose.qx = 0.0;
    front_camera.pose.qy = 0.0;
    front_camera.pose.qz = 0.0;
    front_camera.confidence = 0.95;
    // Camera is 0.3m forward from robot center
    front_camera.camera_offset.x = 0.3;
    front_camera.camera_offset.y = 0.0;
    front_camera.camera_offset.z = 0.0;
    
    printPose(front_camera.pose, "Front Camera Reading");
    std::cout << "  Camera ID: " << front_camera.camera_id << "\n";
    std::cout << "  Confidence: " << front_camera.confidence << "\n\n";
    
    fusion.AddCameraLocalization(front_camera);
    
    // Wait a bit to simulate asynchronous camera updates
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    current_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    
    // Right camera (medium confidence, sees target at an angle)
    CameraLocalization right_camera;
    right_camera.camera_id = 1;
    right_camera.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time);
    right_camera.pose.x = 3.1;
    right_camera.pose.y = -0.1;
    right_camera.pose.z = 1.5;
    right_camera.pose.qw = 1.0;
    right_camera.pose.qx = 0.0;
    right_camera.pose.qy = 0.0;
    right_camera.pose.qz = 0.0;
    right_camera.confidence = 0.80;
    // Camera is 0.2m to the right from robot center
    right_camera.camera_offset.x = 0.0;
    right_camera.camera_offset.y = 0.2;
    right_camera.camera_offset.z = 0.0;
    
    printPose(right_camera.pose, "Right Camera Reading");
    std::cout << "  Camera ID: " << right_camera.camera_id << "\n";
    std::cout << "  Confidence: " << right_camera.confidence << "\n\n";
    
    fusion.AddCameraLocalization(right_camera);
    
    // Wait a bit more
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    current_time = std::chrono::high_resolution_clock::now().time_since_epoch();
    
    // Left camera (lower confidence, partial occlusion)
    CameraLocalization left_camera;
    left_camera.camera_id = 2;
    left_camera.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time);
    left_camera.pose.x = 2.9;
    left_camera.pose.y = 0.2;
    left_camera.pose.z = 1.4;
    left_camera.pose.qw = 1.0;
    left_camera.pose.qx = 0.0;
    left_camera.pose.qy = 0.0;
    left_camera.pose.qz = 0.0;
    left_camera.confidence = 0.60;
    // Camera is 0.2m to the left from robot center
    left_camera.camera_offset.x = 0.0;
    left_camera.camera_offset.y = -0.2;
    left_camera.camera_offset.z = 0.0;
    
    printPose(left_camera.pose, "Left Camera Reading");
    std::cout << "  Camera ID: " << left_camera.camera_id << "\n";
    std::cout << "  Confidence: " << left_camera.confidence << "\n\n";
    
    fusion.AddCameraLocalization(left_camera);
    
    std::cout << "Buffer size: " << fusion.GetBufferSize() << " localizations\n\n";
    
    // Fuse all camera readings
    Pose3D fused_pose;
    bool success = fusion.FusePoses(fused_pose, std::chrono::milliseconds(50));
    
    if (success) {
        std::cout << "=== Fusion Result ===\n";
        printPose(fused_pose, "Fused Pose (weighted average)");
        std::cout << "\nThe fused pose combines all 3 camera readings,\n";
        std::cout << "weighted by their confidence and recency.\n";
        std::cout << "Front camera (95% confidence) has the highest influence.\n";
        std::cout << "Camera offsets have been accounted for.\n\n";
        
        std::cout << "Expected position around [3.0, 0.0, 1.5] with variations\n";
        std::cout << "from the weighted contributions of each camera.\n";
    } else {
        std::cout << "Fusion failed!\n";
        return 1;
    }
    
    std::cout << "\n=== Performance Test ===\n";
    
    // Clear and add many measurements
    fusion.Clear();
    std::cout << "Adding 100 measurements...\n";
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 100; ++i) {
        CameraLocalization loc;
        loc.camera_id = i % 3;
        loc.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now().time_since_epoch());
        loc.pose.x = 3.0 + (rand() % 100) / 1000.0;
        loc.pose.y = 0.0 + (rand() % 100) / 1000.0;
        loc.pose.z = 1.5 + (rand() % 100) / 1000.0;
        loc.pose.qw = 1.0;
        loc.confidence = 0.7 + (rand() % 30) / 100.0;
        
        fusion.AddCameraLocalization(loc);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    std::cout << "Time to add 100 measurements: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() 
              << " ms\n";
    
    std::cout << "Buffer size after 100 additions: " << fusion.GetBufferSize() 
              << " (limited by max buffer size)\n";
    
    start = std::chrono::high_resolution_clock::now();
    success = fusion.FusePoses(fused_pose, std::chrono::milliseconds(50));
    elapsed = std::chrono::high_resolution_clock::now() - start;
    
    if (success) {
        std::cout << "Fusion time: " 
                  << std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() 
                  << " microseconds\n";
        printPose(fused_pose, "\nFinal Fused Pose");
    }
    
    std::cout << "\n=== Demo Complete ===\n";
    
    return 0;
}
