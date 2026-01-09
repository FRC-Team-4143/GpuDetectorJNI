cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 20)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 87)
endif()

if(NOT DEFINED CMAKE_CUDA20_STANDARD_COMPILE_OPTION)
  set(CMAKE_CUDA20_STANDARD_COMPILE_OPTION "-std=c++20")
  set(CMAKE_CUDA20_EXTENSION_COMPILE_OPTION "-std=c++20")
endif()

set(CMAKE_CUDA_FLAGS "--expt-relaxed-constexpr")

# set(CMAKE_BUILD_TYPE DEBUG)

project(971AprilTag LANGUAGES CXX CUDA )
find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(JNI REQUIRED)
find_package(Java REQUIRED)
include(UseJava)
find_package(wpilib REQUIRED)

add_compile_options(-Wno-psabi)

include_directories( . /usr/local/cuda/include ${JNI_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library( 971apriltag SHARED
	frc971/orin/971apriltag.cu
        frc971/orin/apriltag_detect.cu
        frc971/orin/labeling_allegretti_2019_BKE.cu
        frc971/orin/line_fit_filter.cu
        frc971/orin/points.cu
        frc971/orin/threshold.cu
        frc971/orin/cuda.cc
        frc971/orin/pose_fusion.cc
        frc971/orin/camera_pose_estimator.cc
	GpuDetectorJNI.cc
	)
target_link_libraries(971apriltag PRIVATE ${JNI_LIBRARIES}
	${CMAKE_SOURCE_DIR}/third_party/apriltag/build/libapriltag.a wpiutil
	${OpenCV_LIBS}
	)

