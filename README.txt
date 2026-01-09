This is 971's cuda apriltag library wrapped for photonvision.

Features:
- CUDA-accelerated AprilTag detection
- Multi-camera pose fusion for enhanced localization
- Camera pose estimation using Perspective-n-Point (PnP) algorithm
- JNI interface for Java integration

Multi-Camera Pose Fusion:
This library includes a robust multi-camera pose fusion system that combines
AprilTag detections from multiple cameras into a unified pose estimation.
See POSE_FUSION_API.md for detailed API documentation and usage examples.

Camera Pose Estimation:
Estimates camera position and orientation in 3D space using detected AprilTags
and their known world positions. Uses OpenCV's PnP solver with lens distortion
correction. See CAMERA_POSE_ESTIMATION_API.md for detailed documentation.

to compile on a fresh jetpack 6.2 install:
sudo apt install openjdk-17-jdk

#add the following 3 lines to .bashrc
export PATH=$PATH:/usr/local/cuda/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
export JAVA_HOME=/usr/lib/jvm/java-17-openjdk-arm64/


git clone https://github.com/wpilibsuite/allwpilib.git
cd allwpilib
sudo apt install openjdk-17-jdk
sudo apt install ninja-build
sudo apt install protobuf-compiler
sudo apt install libxrandr-dev
sudo apt install libssh-dev
sudo apt install libopencv4.5-java
cmake --preset default -DWITH_GUI=OFF -DWITH_JAVA=ON -DWITH_SIMULATION_MODULES=OFF -DWITH_TESTS=OFF -DOPENCV_JAR_FILE=/usr/share/java/opencv.jar
cd build-cmake 
cmake --build . --parallel 4  #may run out of memory compiling wpimath if too high
sudo cmake --build . --target install

cd ..
git clone https://github.com/FRC-Team-4143/GpuDetectorJNI.git
cd GpuDetectorJNI
cd third_party/apriltag
mkdir build
cd build
cmake ..
make
cd ../../..
mkdir build
cd build
cmake ..
make
sudo cp lib971apriltag.so /usr/lib 

Testing:
To test the pose fusion functionality:
cd GpuDetectorJNI
g++ -std=c++20 -I. test_pose_fusion.cc frc971/orin/pose_fusion.cc -o test_pose_fusion
./test_pose_fusion

To run the demo:
g++ -std=c++20 -I. demo_pose_fusion.cc frc971/orin/pose_fusion.cc -o demo_pose_fusion
./demo_pose_fusion
