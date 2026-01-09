#include <jni.h>
#include <wpi/jni_util.h>
#include <opencv2/core/mat.hpp>

#include <cstdio>
#include <cstring>

#include "frc971/orin/971apriltag.h"
#include "frc971/orin/pose_fusion.h"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/tag36h11.h"
static JavaVM* jvm = nullptr;

static wpi::java::JClass detectionCls;
static wpi::java::JException illegalArgEx;
static wpi::java::JException nullPointerEx;

static const wpi::java::JClassInit classes[] = {
    {"edu/wpi/first/apriltag/AprilTagDetection", &detectionCls}};

static const wpi::java::JExceptionInit exceptions[] = {
    {"java/lang/IllegalArgumentException", &illegalArgEx},
    {"java/lang/NullPointerException", &nullPointerEx}};

static frc971::apriltag::GpuDetector *gpu_detector_ = nullptr; 
static apriltag_detector_t *apriltag_detector_ = nullptr;

frc971::apriltag::CameraMatrix create_camera_matrix() {
  return frc971::apriltag::CameraMatrix{
      1,
      1,
      1,
      1,
  };
}

frc971::apriltag::DistCoeffs create_distortion_coefficients() {
  return frc971::apriltag::DistCoeffs{
      0, 0, 0, 0, 0,
  };
}

struct detectorstruct {
	frc971::apriltag::GpuDetector *gpu_detector_; 
	apriltag_detector_t *apriltag_detector_;
	frc971::apriltag::CameraMatrix camera_matrix;
	frc971::apriltag::DistCoeffs dist_coeffs;
};

detectorstruct detectors[10];
int maxdetectors = 0;

// makes a tag detector.
apriltag_detector_t *maketagdetector(apriltag_family_t *tag_family) {
  apriltag_detector_t *tag_detector = apriltag_detector_create();

  apriltag_detector_add_family_bits(tag_detector, tag_family, 1);

  tag_detector->nthreads = 6;
  tag_detector->wp = workerpool_create(tag_detector->nthreads);
  tag_detector->qtp.min_white_black_diff = 5;
  tag_detector->debug = false;

  return tag_detector;
}

//
// conversions from c++ to java objects
//

static jobject MakeJObject(JNIEnv* env, const apriltag_detection_t* detect) {
  static jmethodID constructor = env->GetMethodID(
      detectionCls, "<init>", "(Ljava/lang/String;IIF[DDD[D)V");
  if (!constructor) {
    return nullptr;
  }

  wpi::java::JLocal<jstring> fam{env, wpi::java::MakeJString(env, detect->family->name)};

  auto homography = detect->H;
  wpi::java::JLocal<jdoubleArray> harr{
      env, wpi::java::MakeJDoubleArray(
               env, {reinterpret_cast<const jdouble*>(homography->data),
                     homography->nrows*homography->ncols})};

  auto corners = detect->p;
  wpi::java::JLocal<jdoubleArray> carr{
      env,
      wpi::java::MakeJDoubleArray(env, {reinterpret_cast<const jdouble*>(corners),
                             4*2})};

  auto center = detect->c;

  return env->NewObject(detectionCls, constructor, fam.obj(),
                        static_cast<jint>(detect->id),
                        static_cast<jint>(detect->hamming),
                        static_cast<jfloat>(detect->decision_margin),
                        harr.obj(), static_cast<jdouble>(center[0]),
                        static_cast<jdouble>(center[1]), carr.obj());
}

static jobjectArray MakeJObject(JNIEnv* env, const zarray_t *detections) {
  jobjectArray jarr = env->NewObjectArray(zarray_size(detections), detectionCls, nullptr);
  if (!jarr) {
    return nullptr;
  }
  if( zarray_size(detections))
    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t *gpu_detection;
      zarray_get(detections, i, &gpu_detection);
      wpi::java::JLocal<jobject> elem{env, MakeJObject(env, gpu_detection)};
      env->SetObjectArrayElement(jarr, i, elem.obj());
    }
  return jarr;
}



extern "C" {

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
  jvm = vm;

  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
    return JNI_ERR;
  }

  // cache references to classes
  for (auto& c : classes) {
    *c.cls = wpi::java::JClass(env, c.name);
    if (!*c.cls) {
      std::fprintf(stderr, "could not load class %s\n", c.name);
      return JNI_ERR;
    }
  }

  for (auto& c : exceptions) {
    *c.cls = wpi::java::JException(env, c.name);
    if (!*c.cls) {
      std::fprintf(stderr, "could not load exception %s\n", c.name);
      return JNI_ERR;
    }
  }

  return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {
  JNIEnv* env;
  if (vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK) {
    return;
  }
  // delete global references
  for (auto& c : classes) {
    c.cls->free(env);
  }
  for (auto& c : exceptions) {
    c.cls->free(env);
  }
  jvm = nullptr;
}

JNIEXPORT jlong JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_createGpuDetector(JNIEnv * jenv, jobject jobj, jint width, jint height) {
    std::cout << "creategpudetector " << width << "x" << height << std::endl;

    if (maxdetectors > 9) {
            std::cout << "creategpudetector too many detectors" << std::endl;
	    return -1;
    }
    detectorstruct detector;
    detector.camera_matrix = create_camera_matrix();
    detector.dist_coeffs = create_distortion_coefficients();
    detector.apriltag_detector_ = maketagdetector(tag36h11_create());
    detector.gpu_detector_ = 
	    new frc971::apriltag::GpuDetector(width, height, 
	   	 detector.apriltag_detector_, 
		 detector.camera_matrix, detector.dist_coeffs);

    detectors[maxdetectors] = detector;
    return maxdetectors++;
}

JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_setparams(JNIEnv * jenv, jobject jobj, jlong handle, jdouble fx, jdouble cx, jdouble fy, jdouble cy, jdouble k1, jdouble k2, jdouble p1, jdouble p2, jdouble k3) {
    std::cout << "setparams" << std::endl;
    if(!detectors[handle].gpu_detector_) {
        std::cout << "971 library lost gpu_detector ptr setparams" << std::endl;
        return;
    }
    int cols = detectors[handle].gpu_detector_->width();
    int rows =  detectors[handle].gpu_detector_->height();
    std::cout << "fx " << fx << " cx " << cx << " fy " << fy << " cy " << cy << std::endl;
    std::cout << "k1 " << k1 << " k2 " << k2 << " p1 " << p1 << " p2 " << p2 << " k3 " << k3 << std::endl;
    detectors[handle].camera_matrix = frc971::apriltag::CameraMatrix{fx, cx, fy, cy};
    detectors[handle].dist_coeffs = frc971::apriltag::DistCoeffs{ k1, k2, p1, p2, k3};
    delete(detectors[handle].gpu_detector_);
    detectors[handle].gpu_detector_ = new frc971::apriltag::GpuDetector(cols, rows, 
    detectors[handle].apriltag_detector_, 
    detectors[handle].camera_matrix, detectors[handle].dist_coeffs);
}

JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_destroyGpuDetector(JNIEnv * jenv, jobject jobj, jlong handle) {
    std::cout << "destroygpudetector" << std::endl;
    if(!detectors[handle].gpu_detector_) {
        std::cout << "971 library lost gpu_detector ptr" << std::endl;
        return;
    }
    delete(detectors[handle].gpu_detector_);
    detectors[handle].gpu_detector_ = nullptr;
    delete(detectors[handle].apriltag_detector_);
    detectors[handle].apriltag_detector_ = nullptr;
}

JNIEXPORT jobjectArray JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_processimage(JNIEnv * jenv, jobject jobj, jlong handle, jlong p) {
	if(!p) {
             std::cout << "971 library got a zero pointer" << std::endl;
	     return nullptr;
	}
	cv::Mat& img  = *(cv::Mat*)p;
	if(!img.ptr()) {
             std::cout << "971 library got a zero pointer in mat" << std::endl;
	     return nullptr;
	}
	if(!detectors[handle].gpu_detector_) {
             std::cout << "971 library lost gpu_detector ptr" << std::endl;
	     return nullptr;
	} else if(img.cols != detectors[handle].gpu_detector_->width() || 
		img.rows != detectors[handle].gpu_detector_->height()) {
             std::cout << "971 library image size mismatch" << std::endl;
             delete(detectors[handle].gpu_detector_);
             detectors[handle].gpu_detector_ = new frc971::apriltag::GpuDetector(img.cols, img.rows, 
	    detectors[handle].apriltag_detector_, 
	    detectors[handle].camera_matrix, detectors[handle].dist_coeffs);
	}
        detectors[handle].gpu_detector_->DetectGrayHost((unsigned char *)img.ptr());

        const zarray_t *detections = detectors[handle].gpu_detector_->Detections();
        if( zarray_size(detections))
        for (int i = 0; i < zarray_size(detections); ++i) {
             //std::cout << "971 library has " << zarray_size(detections) << " detections" << std::endl;
             apriltag_detection_t *gpu_detection;
             zarray_get(detections, i, &gpu_detection);
             std::cout << "tag: " << gpu_detection->id << " at " <<
                     gpu_detection->c[0] << "," << gpu_detection->c[1]
                     << std::endl;
        }
	return MakeJObject(jenv, detections);
}

// Global pose fusion instances (up to 10 fusion objects)
static frc971::apriltag::PoseFusion* pose_fusions[10] = {nullptr};
static int max_pose_fusions = 0;

JNIEXPORT jlong JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_createPoseFusion(JNIEnv* jenv, jobject jobj) {
    if (max_pose_fusions >= 10) {
        std::cout << "createPoseFusion: too many pose fusion objects" << std::endl;
        return -1;
    }
    
    pose_fusions[max_pose_fusions] = new frc971::apriltag::PoseFusion();
    std::cout << "Created pose fusion object: " << max_pose_fusions << std::endl;
    return max_pose_fusions++;
}

JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_destroyPoseFusion(JNIEnv* jenv, jobject jobj, jlong handle) {
    if (handle < 0 || handle >= 10 || !pose_fusions[handle]) {
        std::cout << "destroyPoseFusion: invalid handle" << std::endl;
        return;
    }
    
    delete pose_fusions[handle];
    pose_fusions[handle] = nullptr;
    std::cout << "Destroyed pose fusion object: " << handle << std::endl;
}

JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_addCameraLocalization(
    JNIEnv* jenv, jobject jobj, jlong handle, 
    jint camera_id, jlong timestamp_ns,
    jdouble x, jdouble y, jdouble z,
    jdouble qw, jdouble qx, jdouble qy, jdouble qz,
    jdouble confidence,
    jdouble offset_x, jdouble offset_y, jdouble offset_z,
    jdouble offset_qw, jdouble offset_qx, jdouble offset_qy, jdouble offset_qz) {
    
    if (handle < 0 || handle >= 10 || !pose_fusions[handle]) {
        std::cout << "addCameraLocalization: invalid handle" << std::endl;
        return;
    }
    
    frc971::apriltag::CameraLocalization loc;
    loc.camera_id = camera_id;
    loc.timestamp = std::chrono::nanoseconds(timestamp_ns);
    loc.pose.x = x;
    loc.pose.y = y;
    loc.pose.z = z;
    loc.pose.qw = qw;
    loc.pose.qx = qx;
    loc.pose.qy = qy;
    loc.pose.qz = qz;
    loc.confidence = confidence;
    loc.camera_offset.x = offset_x;
    loc.camera_offset.y = offset_y;
    loc.camera_offset.z = offset_z;
    loc.camera_offset.qw = offset_qw;
    loc.camera_offset.qx = offset_qx;
    loc.camera_offset.qy = offset_qy;
    loc.camera_offset.qz = offset_qz;
    
    pose_fusions[handle]->AddCameraLocalization(loc);
}

JNIEXPORT jdoubleArray JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_fusePoses(
    JNIEnv* jenv, jobject jobj, jlong handle, jlong time_window_ns) {
    
    if (handle < 0 || handle >= 10 || !pose_fusions[handle]) {
        std::cout << "fusePoses: invalid handle" << std::endl;
        return nullptr;
    }
    
    frc971::apriltag::Pose3D fused_pose;
    bool success = pose_fusions[handle]->FusePoses(
        fused_pose, std::chrono::nanoseconds(time_window_ns));
    
    if (!success) {
        return nullptr;
    }
    
    // Return array of [x, y, z, qw, qx, qy, qz]
    jdoubleArray result = jenv->NewDoubleArray(7);
    if (result == nullptr) {
        return nullptr;
    }
    
    jdouble pose_data[7] = {
        fused_pose.x, fused_pose.y, fused_pose.z,
        fused_pose.qw, fused_pose.qx, fused_pose.qy, fused_pose.qz
    };
    
    jenv->SetDoubleArrayRegion(result, 0, 7, pose_data);
    
    std::cout << "Fused pose: [" << fused_pose.x << ", " << fused_pose.y << ", " 
              << fused_pose.z << "] quat: [" << fused_pose.qw << ", " 
              << fused_pose.qx << ", " << fused_pose.qy << ", " 
              << fused_pose.qz << "]" << std::endl;
    
    return result;
}

JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_clearPoseFusion(JNIEnv* jenv, jobject jobj, jlong handle) {
    if (handle < 0 || handle >= 10 || !pose_fusions[handle]) {
        std::cout << "clearPoseFusion: invalid handle" << std::endl;
        return;
    }
    
    pose_fusions[handle]->Clear();
    std::cout << "Cleared pose fusion buffer: " << handle << std::endl;
}

JNIEXPORT jint JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_getPoseFusionBufferSize(
    JNIEnv* jenv, jobject jobj, jlong handle) {
    
    if (handle < 0 || handle >= 10 || !pose_fusions[handle]) {
        std::cout << "getPoseFusionBufferSize: invalid handle" << std::endl;
        return -1;
    }
    
    return static_cast<jint>(pose_fusions[handle]->GetBufferSize());
}

JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_setPoseFusionMaxBufferSize(
    JNIEnv* jenv, jobject jobj, jlong handle, jint max_size) {
    
    if (handle < 0 || handle >= 10 || !pose_fusions[handle]) {
        std::cout << "setPoseFusionMaxBufferSize: invalid handle" << std::endl;
        return;
    }
    
    pose_fusions[handle]->SetMaxBufferSize(static_cast<size_t>(max_size));
    std::cout << "Set max buffer size to " << max_size << " for fusion object: " << handle << std::endl;
}

} // extern "C"

