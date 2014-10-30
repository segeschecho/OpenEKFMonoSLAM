#Copyright (C) 2013 Sergio E. Gonzalez and Emiliano D. González
#Facultad de Ciencias Exactas y Naturales, Universidad de Buenos Aires, Buenos Aires, Argentina.
 
#C/C++, Java and XML/YML code for EKF SLAM from a monocular sequence.

#This file is part of OpenEKFMonoSLAM.

#OpenEKFMonoSLAM is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#OpenEKFMonoSLAM is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with OpenEKFMonoSLAM.  If not, see <http:www.gnu.org/licenses/>.

#If you use this code for academic work, please reference:
#GONZALEZ, S.; E. GONZÁLEZ; M. NITSCHE; P. DE CRISTÓFORIS. "Odometría Visual para Robots Móviles Utilizando Smartphones como Unidad de Sensado y Procesamiento". En: Jornadas Argentinas de Robótica, 8as : 2014 : Ciudad Autónoma de Buenos Aires. Actas : VIII Jornadas Argentinas de Robótica . (8 : Ciudad Autónoma de Buenos Aires, 12-14 de noviembre 2014).

#Authors:    Sergio E. Gonzalez - segonzalez@dc.uba.ar
#            Emiliano D. González - edgonzalez@dc.uba.ar

#Departamento de Computación
#Facultad de Ciencias Exactas y Naturales
#Universidad de Buenos, Buenos Aires, Argentina
#Date: June 2013

LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/OpenCV-2.4.3.2-android-sdk/sdk/native/jni/OpenCV.mk
include $(call all-subdir-makefiles)
#include $(CLEAR_VARS)

# Configuracion OpenCV
OPENCV_CAMERA_MODULES := off
OPENCV_INSTALL_MODULES := off

# Configuracion de nuestras fuentes
LOCAL_C_INCLUDES += $(LOCAL_PATH)/EKF/modules/1PointRansacEKF $(LOCAL_PATH)/EKF/modules/Core $(LOCAL_PATH)/EKF/modules/ImageGenerator $(LOCAL_PATH)/EKF/modules/Matching $(LOCAL_PATH)/EKF/modules/MeasurementPrediction $(LOCAL_PATH)/EKF/modules/ $(LOCAL_PATH)/EKF/ $(LOCAL_PATH)

LOCAL_LDLIBS += -llog

LOCAL_MODULE := EKFNative
LOCAL_SRC_FILES := EKFNative.cpp Handler.cpp EKF/modules/Core/EKFMath.cpp EKF/modules/Core/Timer.cpp EKF/modules/Gui/Draw.cpp EKF/modules/1PointRansacEKF/1PointRansac.cpp EKF/modules/1PointRansacEKF/DetectNewImageFeatures.cpp EKF/modules/1PointRansacEKF/Matching.cpp EKF/modules/1PointRansacEKF/State.cpp EKF/modules/1PointRansacEKF/ImageFeatureMeasurement.cpp EKF/modules/1PointRansacEKF/AddRemoveMapFeature.cpp EKF/modules/1PointRansacEKF/EKF.cpp EKF/modules/1PointRansacEKF/ImageFeaturePrediction.cpp EKF/modules/1PointRansacEKF/MeasurementPrediction.cpp EKF/modules/1PointRansacEKF/Update.cpp EKF/modules/1PointRansacEKF/CommonFunctions.cpp EKF/modules/1PointRansacEKF/MapFeature.cpp EKF/modules/1PointRansacEKF/StateAndCovariancePrediction.cpp EKF/modules/1PointRansacEKF/HandMatching.cpp EKF/modules/Configuration/ConfigurationManager.cpp EKF/modules/Configuration/ConfigurationDataReader/ConfigurationDataReader.cpp EKF/modules/Configuration/ConfigurationDataReader/CameraCalibrationConfiguration/CameraCalibrationConfiguration.cpp EKF/modules/Configuration/ConfigurationDataReader/DescriptorExtractorConfiguration/DescriptorExtractorConfiguration.cpp EKF/modules/Configuration/ConfigurationDataReader/DescriptorExtractorConfiguration/DescriptorExtractorFactory.cpp EKF/modules/Configuration/ConfigurationDataReader/ExtendedKalmanFilterConfiguration/ExtendedKalmanFilterConfiguration.cpp EKF/modules/Configuration/ConfigurationDataReader/FeatureDetectorConfiguration/FeatureDetectorConfiguration.cpp EKF/modules/Configuration/ConfigurationDataReader/FeatureDetectorConfiguration/FeatureDetectorFactory.cpp EKF/modules/1PointRansacEKF/MapManagement.cpp EKF/modules/ImageGenerator/CameraImageGenerator.cpp EKF/modules/ImageGenerator/FileSequenceImageGenerator.cpp EKF/modules/ImageGenerator/SlidingWindowImageGenerator.cpp EKF/modules/ImageGenerator/VideoFileImageGenerator.cpp EKF/modules/ImageGenerator/FileSequenceOnDemandImageGenerator.cpp GenericFunctions.cpp

include $(BUILD_SHARED_LIBRARY)
