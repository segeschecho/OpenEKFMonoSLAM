//Copyright (C) 2013 Sergio E. Gonzalez and Emiliano D. González
//Facultad de Ciencias Exactas y Naturales, Universidad de Buenos Aires, Buenos Aires, Argentina.
// 
//C/C++, Java and XML/YML code for EKF SLAM from a monocular sequence.
//
//This file is part of OpenEKFMonoSLAM.
//
//OpenEKFMonoSLAM is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//OpenEKFMonoSLAM is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with OpenEKFMonoSLAM.  If not, see <http:www.gnu.org/licenses/>.
//
//If you use this code for academic work, please reference:
//GONZALEZ, S.; E. GONZÁLEZ; M. NITSCHE; P. DE CRISTÓFORIS. "Odometría Visual para Robots Móviles Utilizando Smartphones como Unidad de Sensado y Procesamiento". En: Jornadas Argentinas de Robótica, 8as : 2014 : Ciudad Autónoma de Buenos Aires. Actas : VIII Jornadas Argentinas de Robótica . (8 : Ciudad Autónoma de Buenos Aires, 12-14 de noviembre 2014).
//
//Authors:    Sergio E. Gonzalez - segonzalez@dc.uba.ar
//            Emiliano D. González - edgonzalez@dc.uba.ar
//
//Departamento de Computación
//Facultad de Ciencias Exactas y Naturales
//Universidad de Buenos, Buenos Aires, Argentina
//Date: June 2013

#include <string.h>
#include <iostream>
#include <jni.h>

#include <android/log.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Handler.h"
#include "GenericFunctions.h"

#include "EKF/modules/Core/Base.h"
#include "EKF/modules/Configuration/ConfigurationManager.h"
#include "EKF/modules/1PointRansacEKF/State.h"
#include "EKF/modules/1PointRansacEKF/EKF.h"
#include "EKF/modules/Gui/Draw.h"


using namespace cv;

// ------------------------------------------------------------------------------------------
// Macros
// ------------------------------------------------------------------------------------------
#define ANDROID_DEBUG 1

// ------------------------------------------------------------------------------------------
// Funciones que se exportan
// ------------------------------------------------------------------------------------------
extern "C" {
    JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_loadEKFNativeReference(JNIEnv* env, jobject object,
                                                                         jstring configurationFilePath,
                                                                         jstring outputPath);
    JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_loadEKFCameraPreviewDimensions(JNIEnv* env, jobject object);
    JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_EKFInit(JNIEnv* env, jobject object,
                                                              jbyteArray imageData, jint imageWidth, jint imageHeight);
    JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_EKFStep(JNIEnv* env, jobject object,
                                                              jbyteArray imageData, jint imageWidth, jint imageHeight,
                                                              jintArray imageWithPredictionInfo);
    JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_releaseEKFNativeReference(JNIEnv* env, jobject object);
}


// ------------------------------------------------------------------------------------------
// loadEKFNativeReference: 
// ------------------------------------------------------------------------------------------
JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_loadEKFNativeReference(JNIEnv* env, jobject object,
                                                                         jstring configurationFilePath,
                                                                         jstring outputPath)
{
    // Se crea la instancia del EKF y se setea su identificador del lado de Java
    const char *_configurationFilePath = env->GetStringUTFChars(configurationFilePath, 0);
    const char *_outputPath = env->GetStringUTFChars(outputPath, 0);

#ifdef ANDROID_DEBUG
    __android_log_print(ANDROID_LOG_ERROR, "loadEKFNativeReference", "El archivo de configuracion esta en: %s", _configurationFilePath);
#endif

    EKF *ekfInstance = new EKF(_configurationFilePath, _outputPath);

    setHandle(env, object, ekfInstance);

    env->ReleaseStringUTFChars(configurationFilePath, _configurationFilePath);
    env->ReleaseStringUTFChars(outputPath, _outputPath);
}

// Get the Field ID of the instance variable to be accessed via GetFieldID() from the class reference.
// You need to provide the variable name and its field descriptor (or signature).
// For a Java class, the field descriptor is in the form of "L<fully-qualified-name>;", with dot replaced by forward slash (/), e.g.,, the class descriptor for String is "Ljava/lang/String;".
// For primitives, use "I" for int, "B" for byte, "S" for short, "J" for long, "F" for float, "D" for double, "C" for char, and "Z" for boolean. For arrays, include a prefix "[", e.g., "[Ljava/lang/Object;" for an array of Object; "[I" for an array of int.

JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_loadEKFCameraPreviewDimensions(JNIEnv* env, jobject object)
{
    // Se setean las dimensiones de la imagen con las que trabaja el EKF
    jclass c = env->GetObjectClass(object);

    jfieldID cameraPreviewWidthId = env->GetFieldID(c, "_cameraPreviewWidth", "I");
    jfieldID cameraPreviewHeightId = env->GetFieldID(c, "_cameraPreviewHeight", "I");

    int _camPreviewWidth = env->GetIntField(object, cameraPreviewWidthId);
    int _camPreviewHeight = env->GetIntField(object, cameraPreviewHeightId);

    _camPreviewWidth = ConfigurationManager::getInstance().cameraCalibration->pixelsX;
    _camPreviewHeight = ConfigurationManager::getInstance().cameraCalibration->pixelsY;

    env->SetIntField(object, cameraPreviewWidthId, _camPreviewWidth);
    env->SetIntField(object, cameraPreviewHeightId, _camPreviewHeight);

}

// ------------------------------------------------------------------------------------------
// EKFInit: Funcion que inicia todos los parametros para el EKF y procesa la primer imagen.
// ------------------------------------------------------------------------------------------
JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_EKFInit(JNIEnv* env, jobject object,
                                                          jbyteArray imageData, jint imageWidth, jint imageHeight)
{
    jboolean isCopy = JNI_TRUE;
    jbyte* _imageData  = env->GetByteArrayElements(imageData, &isCopy);

    cv::Mat image(imageHeight, imageWidth, CV_8UC1, (unsigned char *)_imageData);

    cv::Mat myuv(imageHeight + imageHeight/2, imageWidth, CV_8UC1, (unsigned char *)_imageData);
    cv::Mat mrgba(imageHeight, imageWidth, CV_8UC4);

    cvtColor(myuv, mrgba, 90 /*CV_YUV420i2RGB*/ , 3);

#ifdef ANDROID_DEBUG
    __android_log_print(ANDROID_LOG_ERROR, "EKF Init", "Se esta por hacer Init!!");
#endif
    EKF *ekf = getHandle(env, object);
    ekf->init(mrgba);

#ifdef ANDROID_DEBUG
    __android_log_print(ANDROID_LOG_ERROR, "EKF Init", "Se hizo ekfInit !! ........"); 
#endif

    env->ReleaseByteArrayElements(imageData, _imageData, 0);
}
// ------------------------------------------------------------------------------------------
// EKFStep: Funcion que calcula un paso del EKF.
// ------------------------------------------------------------------------------------------

JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_EKFStep(JNIEnv* env, jobject object,
                                                          jbyteArray imageData, jint imageWidth, jint imageHeight,
                                                          jintArray imageWithPredictionInfo)
{
    jboolean isCopy = JNI_TRUE;
    jbyte* _imageData  = env->GetByteArrayElements(imageData, &isCopy);
    // cv::Mat image(imageHeight, imageWidth, CV_8UC1, (unsigned char *)_imageData);

    cv::Mat myuv(imageHeight + imageHeight/2, imageWidth, CV_8UC1, (unsigned char *)_imageData);
    cv::Mat mrgba(imageHeight, imageWidth, CV_8UC4);

    cvtColor(myuv, mrgba, 90 /*CV_YUV420i2RGB*/ , 3);

#ifdef ANDROID_DEBUG
    __android_log_print(ANDROID_LOG_ERROR, "EKF Step", "Se va a llamar a EKFStep");
#endif

    EKF *ekf = getHandle(env, object);
    ekf->step(mrgba);

#ifdef ANDROID_DEBUG
    __android_log_print(ANDROID_LOG_ERROR, "EKF Step", "Se hizo EKFStep !!!");
    __android_log_print(ANDROID_LOG_ERROR, "EKF Step", "Features en el mapa: %d", ekf->state.mapFeatures.size());
#endif

    // Se actualiza la posicion de la camara dentro de la clase java
    jclass c = env->GetObjectClass(object);

    jfieldID xCamId = env->GetFieldID(c, "xCam", "D");
    jfieldID yCamId = env->GetFieldID(c, "yCam", "D");
    jfieldID zCamId = env->GetFieldID(c, "zCam", "D");

    double _xCam = env->GetDoubleField(object, xCamId);
    double _yCam = env->GetDoubleField(object, yCamId);
    double _zCam = env->GetDoubleField(object, zCamId);

    _xCam = ekf->state.position[0];
    _yCam = ekf->state.position[1];
    _zCam = ekf->state.position[2];

#ifdef ANDROID_DEBUG
    __android_log_print(ANDROID_LOG_ERROR, "EKF Step", "Posicion de la camara: %f %f %f", _xCam, _yCam, _zCam);
#endif

    env->SetDoubleField(object, xCamId, _xCam);
    env->SetDoubleField(object, yCamId, _yCam);
    env->SetDoubleField(object, zCamId, _zCam);

    env->ReleaseByteArrayElements(imageData, _imageData, 0);
}

// ------------------------------------------------------------------------------------------
// Release: Funcion que libera las estructuras de todo.
// ------------------------------------------------------------------------------------------
JNIEXPORT void JNICALL Java_tesis_ekfmonoslam_EKF_releaseEKFNativeReference(JNIEnv* env, jobject object)
{
    EKF *p = getHandle(env, object);
    setHandle(env, object, 0);
    delete p;
    
#ifdef ANDROID_DEBUG
    __android_log_print(ANDROID_LOG_ERROR, "releaseEKFNativeReference", "Se libera la estructura EKF nativa");
#endif
}
