#include <jni.h>
#include <wpi/jni_util.h>
#include <opencv2/core/mat.hpp>

#include <cstdio>
#include <cstring>

#include "frc971/orin/971apriltag.h"
#include "third_party/apriltag/apriltag.h"
#include "third_party/apriltag/tag36h11.h"
static JavaVM* jvm = nullptr;

static wpi::java::JClass detectionCls;
//static wpi::java::JClass detectorConfigCls;
//static wpi::java::JClass detectorQTPCls;
//static wpi::java::JClass poseEstimateCls;
//static wpi::java::JClass quaternionCls;
//static wpi::java::JClass rotation3dCls;
//static wpi::java::JClass transform3dCls;
//static wpi::java::JClass translation3dCls;
//static wpi::java::JClass rawFrameCls;
static wpi::java::JException illegalArgEx;
static wpi::java::JException nullPointerEx;

static const wpi::java::JClassInit classes[] = {
    {"edu/wpi/first/apriltag/AprilTagDetection", &detectionCls}}; //,
    //{"edu/wpi/first/apriltag/AprilTagDetector$Config", &detectorConfigCls},
    //{"edu/wpi/first/apriltag/AprilTagDetector$QuadThresholdParameters", &detectorQTPCls},
    //{"edu/wpi/first/apriltag/AprilTagPoseEstimate", &poseEstimateCls},
    //{"edu/wpi/first/math/geometry/Quaternion", &quaternionCls},
    //{"edu/wpi/first/math/geometry/Rotation3d", &rotation3dCls},
    //{"edu/wpi/first/math/geometry/Transform3d", &transform3dCls},
    //{"edu/wpi/first/math/geometry/Translation3d", &translation3dCls},
    //{"edu/wpi/first/util/RawFrame", &rawFrameCls}};

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

JNIEXPORT jint JNICALL JNI_Onload(JavaVM* vm, void* reserved) {
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
    
    /*if(!apriltag_detector_) 
        apriltag_detector_ = maketagdetector(tag36h11_create());
    if(gpu_detector_ && width == gpu_detector_->width() && height == gpu_detector_->height()) return;
    std::cout << "create new gpudetector" << std::endl;
    delete(gpu_detector_);
    gpu_detector_ = new frc971::apriltag::gpudetector(width, height, 
	    apriltag_detector_, 
	    create_camera_matrix(), create_distortion_coefficients());
    std::cout << "created new gpudetector" << std::endl; */
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
    detectors[handle].camera_matrix = frc971::apriltag::CameraMatrix{fx, cy, fy, cy};
    detectors[handle].dist_coeffs = frc971::apriltag::DistCoeffs{ k1, k2, p1, p2, k3};
    delete(detectors[handle].gpu_detector_);
    detectors[handle].gpu_detector_ = new frc971::apriltag::GpuDetector(cols, rows, 
    detectors[handle].apriltag_detector_, 
    detectors[handle].camera_matrix, detectors[handle].dist_coeffs);
}

JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuDetectorJNI_destroyGpuDetector(JNIEnv * jenv, jobject jobj, jlong handle) {
    std::cout << "destroygpudetector" << std::endl;
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
             std::cout << "971 library has " << zarray_size(detections) << " detections" << std::endl;
             apriltag_detection_t *gpu_detection;
             zarray_get(detections, i, &gpu_detection);
             std::cout << "tag: " << gpu_detection->id << " at " <<
                     gpu_detection->c[0] << "," << gpu_detection->c[1]
                     << std::endl;
        }
	return MakeJObject(jenv, detections);
}

} // extern "C"

