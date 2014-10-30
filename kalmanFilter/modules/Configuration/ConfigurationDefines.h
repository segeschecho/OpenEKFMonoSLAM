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

#ifndef __MODULES_CONFIGURATION_CONFIGURATIONDEFINES_H__
#define __MODULES_CONFIGURATION_CONFIGURATIONDEFINES_H__

#define CONFIG_TRUE_KEY "true"
#define CONFIG_FALSE_KEY "false"

#define CONFIG_RUN_CONFIG_KEY "RunConfiguration"
#define CONFIG_RUN_CONFIG_EKF_KEY "ExtendedKalmanFilter"
#define CONFIG_RUN_CONFIG_CAMERA_CALIB_KEY "CameraCalibration"
#define CONFIG_RUN_CONFIG_FEATURE_DETECTOR_KEY "FeatureDetector"
#define CONFIG_RUN_CONFIG_DESCRIPTOR_EXTRACTOR_KEY "DescriptorExtractor"

#define CONFIG_FEATURE_DETECTOR_KEY "FeatureDetector"
#define CONFIG_FEATURE_DETECTOR_TYPE_KEY "Type"

#ifndef ANDROID
#define CONFIG_FEATURE_DETECTOR_TYPE_SURF_KEY "SURF"
#define CONFIG_FEATURE_DETECTOR_TYPE_SURF_HESSIAN_TRESHOLD_KEY "HessianThreshold"
#define CONFIG_FEATURE_DETECTOR_TYPE_SURF_OCTAVES_KEY "Octaves"
#define CONFIG_FEATURE_DETECTOR_TYPE_SURF_OCTAVE_LAYERS_KEY "OctaveLayers"
#define CONFIG_FEATURE_DETECTOR_TYPE_SURF_EXTENDED_KEY "Extended"
#define CONFIG_FEATURE_DETECTOR_TYPE_SURF_UPRIGHT_KEY "Upright"
#endif

#define CONFIG_FEATURE_DETECTOR_TYPE_FAST_KEY "FAST"
#define CONFIG_FEATURE_DETECTOR_TYPE_FAST_TRESHOLD_KEY "Threshold"
#define CONFIG_FEATURE_DETECTOR_TYPE_FAST_NONMAXSUPPRESSION_KEY "NonmaxSuppression"

#ifndef ANDROID
#define CONFIG_FEATURE_DETECTOR_TYPE_SIFT_KEY "SIFT"
#define CONFIG_FEATURE_DETECTOR_TYPE_SIFT_FEATURES_KEY "Features"
#define CONFIG_FEATURE_DETECTOR_TYPE_SIFT_OCTAVE_LAYERS_KEY "OctaveLayers"
#define CONFIG_FEATURE_DETECTOR_TYPE_SIFT_CONTRAST_THRESHOLD_KEY "ContrastThreshold"
#define CONFIG_FEATURE_DETECTOR_TYPE_SIFT_EDGE_THRESHOLD_KEY "EdgeThreshold"
#define CONFIG_FEATURE_DETECTOR_TYPE_SIFT_SIGMA_KEY "Sigma"
#endif

#define CONFIG_FEATURE_DETECTOR_TYPE_ORB_KEY "ORB"

#define CONFIG_FEATURE_DETECTOR_TYPE_STAR_KEY "STAR"
#define CONFIG_FEATURE_DETECTOR_TYPE_STAR_MAX_SIZE_KEY "MaxSize"
#define CONFIG_FEATURE_DETECTOR_TYPE_STAR_RESPONSE_THRESHOLD_KEY "ResponseThreshold"
#define CONFIG_FEATURE_DETECTOR_TYPE_STAR_LINE_THRESHOLD_PROJECTED_KEY "LineThresholdProjected"
#define CONFIG_FEATURE_DETECTOR_TYPE_STAR_LINE_THRESHOLD_BINARIZED_KEY "LineThresholdBinarized"
#define CONFIG_FEATURE_DETECTOR_TYPE_STAR_SUPPRESSNONMAXSIZE_KEY "SuppressNonmaxSize"

#define CONFIG_DESCRIPTOR_EXTRACTOR_KEY "DescriptorExtractor"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_KEY "Type"

#ifndef ANDROID
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_KEY "SURF"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_HESSIAN_TRESHOLD_KEY "HessianTreshold"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_OCTAVES_KEY "Octaves"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_OCTAVE_LAYERS_KEY "OctaveLayers"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_EXTENDED_KEY "Extended"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_UPRIGHT_KEY "Upright"

#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_KEY "SIFT"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_FEATURES_KEY "Features"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_OCTAVE_LAYERS_KEY "OctaveLayers"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_CONTRAST_THRESHOLD_KEY "ContrastThreshold"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_EDGE_THRESHOLD_KEY "EdgeThreshold"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_SIGMA_KEY "Sigma"
#endif

#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_ORB_KEY "ORB"

#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_BRIEF_KEY "BRIEF"
#define CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_BRIEF_BYTES_LENGTH_KEY "BytesLength:"

// ------------------------------------------------------------------------------------------------------------
// Parametros de la camara:
// ------------------------------------------------------------------------------------------------------------
//  CAMERA_PIXELS_X             Cantidad de pixels en X de la imagen
//  CAMERA_PIXELS_Y             Cantidad de pixels en Y de la imagen
//  CAMERA_INTRINSIC_F          Largo focal en mm
//  CAMERA_INTRINSIC_F_X        Largo focal en X, en pixels
//  CAMERA_INTRINSIC_F_Y        Largo focal en Y, en pixels
//  CAMERA_INTRINSIC_K1         Distorsion radial
//  CAMERA_INTRINSIC_K2         Distorsion tangencial
//  CAMERA_INTRINSIC_CX         Punto principal en X en pixels
//  CAMERA_INTRINSIC_CY         Punto principal en Y en pixels
//  CAMERA_INTRINSIC_DX         Tamano de cada pixel en la ccd de la camara en mm (de centro a centro) en X
//  CAMERA_INTRINSIC_DY         Tamano de cada pixel en la ccd de la camara en mm (de centro a centro) en Y
//  CAMERA_ANGULAR_VISION_X     Angulo de vision en X (grados)
//  CAMERA_ANGULAR_VISION_Y     Angulo de vision en Y (grados)

#define CONFIG_CAMERA_CALIB_KEY "CameraCalibration"

#define CONFIG_CAMERA_PIXELS_X_KEY          "PixelsX"
#define CONFIG_CAMERA_PIXELS_Y_KEY          "PixelsY"
#define CONFIG_CAMERA_INTRINSIC_FX_KEY      "FX"
#define CONFIG_CAMERA_INTRINSIC_FY_KEY      "FY"
#define CONFIG_CAMERA_INTRINSIC_K1_KEY      "K1"
#define CONFIG_CAMERA_INTRINSIC_K2_KEY      "K2"
#define CONFIG_CAMERA_INTRINSIC_CX_KEY      "CX"
#define CONFIG_CAMERA_INTRINSIC_CY_KEY      "CY"
#define CONFIG_CAMERA_INTRINSIC_DX_KEY      "DX"
#define CONFIG_CAMERA_INTRINSIC_DY_KEY      "DY"
#define CONFIG_CAMERA_PIXEL_ERROR_X_KEY     "PixelErrorX"
#define CONFIG_CAMERA_PIXEL_ERROR_Y_KEY     "PixelErrorY"
#define CONFIG_CAMERA_ANGULAR_VISION_X_KEY  "AngularVisionX"
#define CONFIG_CAMERA_ANGULAR_VISION_Y_KEY  "AngularVisionY"

#define CONFIG_EKF_KEY "ExtendedKalmanFilter"

// Parametros generales

#define CONFIG_EKF_FEATURES_STATE_RESERVE_DEPTH_KEY         "ReserveFeaturesDepth"
#define CONFIG_EKF_FEATURES_STATE_RESERVE_INVERSE_DEPTH_KEY "ReserveFeaturesInvDepth"

// Parametros del EKF

#define CONFIG_EKF_INIT_INVDEPTH_RHO_KEY     "InitInvDepthRho"

#define CONFIG_EKF_INIT_LINEAR_ACCEL_SD_KEY   "InitLinearAccelSD"
#define CONFIG_EKF_INIT_ANGULAR_ACCEL_SD_KEY  "InitAngularAccelSD"
#define CONFIG_EKF_LINEAR_ACCEL_SD_KEY        "LinearAccelSD"
#define CONFIG_EKF_ANGULAR_ACCEL_SD_KEY       "AngularAccelSD"
#define CONFIG_EKF_INVERSE_DEPTH_RHO_SD_KEY   "InverseDepthRhoSD"

#define CONFIG_EKF_MAX_MAP_FEATURES_COUNT_KEY "MaxMapFeaturesCount"
#define CONFIG_EKF_MAX_MAP_SIZE_KEY "MaxMapSize"
#define CONFIG_EKF_ALWAYS_REMOVE_UNSEEN_MAPFEATURES_KEY "AlwaysRemoveUnseenMapFeatures"

#define CONFIG_EKF_MIN_MATCHES_PER_IMAGE_KEY "MinMatchesPerImage"

#define CONFIG_EKF_MAP_MANAGEMENT_FREQUENCY_KEY "MapManagementFrequency"

#define CONFIG_EKF_MATCHING_SECOND_BEST_DIST_COMPARE_COEF_KEY "MatchingCompCoefSecondBestVSFirst"

#define CONFIG_EKF_DETECT_NEW_FEATURES_IMAGE_AREAS_DIVIDE_TIMES_KEY "DetectNewFeaturesImageAreasDivideTimes"
#define CONFIG_EKF_DETECT_NEW_FEATURES_IMAGE_MASK_ELLIPSE_SIZE_KEY "DetectNewFeaturesImageMaskEllipseSize"

#define CONFIG_EKF_GOOD_FEATURE_MATCHING_PERCENT_KEY "GoodFeatureMatchingPercent"

#define CONFIG_EKF_INVDEPTH_LINEARITY_INDEX_THRESHOLD "InverseDepthLinearityIndexThreshold"

// Parametros para 1PointRansac

#define CONFIG_EKF_RANSAC_THRESHOLD_PREDICTION_DISTANCE_KEY "RansacThresholdPredictDistance"
#define CONFIG_EKF_RANSAC_SET_ALL_INLIERS_PROBABILITY_KEY   "RansacAllInliersProbability"
#define CONFIG_EKF_RANSAC_CHI2_THRESHOLD_KEY                "RansacChi2Threshold"

#endif // __MODULES_CONFIGURATION_CONFIGURATIONDEFINES_H__
