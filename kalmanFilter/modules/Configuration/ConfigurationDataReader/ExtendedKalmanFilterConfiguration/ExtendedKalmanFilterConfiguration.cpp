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

#include "../../ConfigurationDefines.h"
#include "ExtendedKalmanFilterConfiguration.h"
#include "ExtendedKalmanFilterParameters.h"

// ------------------------------------------------------------------------------------------
// Constructor and Destructor Implementation
// ------------------------------------------------------------------------------------------

ExtendedKalmanFilterConfiguration::ExtendedKalmanFilterConfiguration()
{
}

// ------------------------------------------------------------------------------------------

ExtendedKalmanFilterConfiguration::~ExtendedKalmanFilterConfiguration()
{
    Dictionary< void* >::iterator itMap = _dataMap.begin();
    Dictionary< void* >::iterator itMapEnd = _dataMap.end();
    
    while (itMap != itMapEnd)
    {
        delete static_cast<ExtendedKalmanFilterParameters *>(itMap->second);
        itMap++;
    }
}

// ------------------------------------------------------------------------------------------
// Inherited Methods Implementation
// ------------------------------------------------------------------------------------------

void* ExtendedKalmanFilterConfiguration::loadNodeFromFileNode(const cv::FileNode &extendedKalmanFilterFileNode, const char *alias)
{
    Dictionary<void *>::const_iterator it = _dataMap.find(alias);
    if (it != _dataMap.end())
    {
        std::cerr << "ERROR: EKF configuration with name " << alias << " was already defined.";
        return it->second;
    }

    cv::FileNode ekfParamsNode = extendedKalmanFilterFileNode[alias];
    
    cv::FileNodeIterator itParametersEnd = ekfParamsNode.end();
    cv::FileNodeIterator itParameters = ekfParamsNode.begin();
    
    if (itParameters == itParametersEnd)
    {
        std::cerr << "ERROR: EKF configuration with name " << alias << " was not found.";
        return NULL;
    }

    Dictionary<std::string> parameters;
    while (itParameters != itParametersEnd)
    {
        std::string sParameterName = (*itParameters).name();
        std::string parameterValue = (std::string)(*itParameters);
        
        parameters[sParameterName] = parameterValue;
        
        itParameters++;
    }
    
    ExtendedKalmanFilterParameters* newExtendedKalmanFilterParameters = createNewEKFParameters(alias, parameters);
    _dataMap[alias] = newExtendedKalmanFilterParameters;
    
    return newExtendedKalmanFilterParameters;
}

// ------------------------------------------------------------------------------------------
// Private Methods Implementation
// ------------------------------------------------------------------------------------------

ExtendedKalmanFilterParameters* ExtendedKalmanFilterConfiguration::createNewEKFParameters( std::string ekfParamsName,
                                                                                           const Dictionary<std::string> &parameters ) const
{
    ExtendedKalmanFilterParameters *newEKFParams = new ExtendedKalmanFilterParameters();
    Dictionary<std::string>::const_iterator parametersEnd = parameters.end();
    Dictionary<std::string>::const_iterator parameter;
    
    newEKFParams->ekfParametersName = ekfParamsName;
    
    parameter = parameters.find(CONFIG_EKF_FEATURES_STATE_RESERVE_DEPTH_KEY);
    newEKFParams->reserveFeaturesDepth = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 1024;

    parameter = parameters.find(CONFIG_EKF_FEATURES_STATE_RESERVE_INVERSE_DEPTH_KEY);
    newEKFParams->reserveFeaturesInvDepth = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 1024;
    
    parameter = parameters.find(CONFIG_EKF_MAX_MAP_FEATURES_COUNT_KEY);
    newEKFParams->maxMapFeaturesCount = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 0;
    
    parameter = parameters.find(CONFIG_EKF_MAX_MAP_SIZE_KEY);
    newEKFParams->maxMapSize = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 0;
    
    parameter = parameters.find(CONFIG_EKF_ALWAYS_REMOVE_UNSEEN_MAPFEATURES_KEY);
    newEKFParams->alwaysRemoveUnseenMapFeatures = (parameter != parametersEnd) ?
                                                    (parameter->second.compare( std::string(CONFIG_TRUE_KEY) ) == 0) : false;

    parameter = parameters.find(CONFIG_EKF_INIT_INVDEPTH_RHO_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->initInvDepthRho = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_INIT_LINEAR_ACCEL_SD_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->initLinearAccelSD = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_INIT_ANGULAR_ACCEL_SD_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->initAngularAccelSD = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_EKF_LINEAR_ACCEL_SD_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->linearAccelSD = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_ANGULAR_ACCEL_SD_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->angularAccelSD = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_INVERSE_DEPTH_RHO_SD_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->inverseDepthRhoSD = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_EKF_MAP_MANAGEMENT_FREQUENCY_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->mapManagementFrequency = atoi(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_DETECT_NEW_FEATURES_IMAGE_AREAS_DIVIDE_TIMES_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->detectNewFeaturesImageAreasDivideTimes = atoi(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_EKF_DETECT_NEW_FEATURES_IMAGE_MASK_ELLIPSE_SIZE_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->detectNewFeaturesImageMaskEllipseSize = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_MATCHING_SECOND_BEST_DIST_COMPARE_COEF_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->matchingCompCoefSecondBestVSFirst = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_MIN_MATCHES_PER_IMAGE_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->minMatchesPerImage = atoi(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_GOOD_FEATURE_MATCHING_PERCENT_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->goodFeatureMatchingPercent = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_EKF_RANSAC_THRESHOLD_PREDICTION_DISTANCE_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->ransacThresholdPredictDistance = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_RANSAC_SET_ALL_INLIERS_PROBABILITY_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->ransacAllInliersProbability = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_RANSAC_CHI2_THRESHOLD_KEY);
    assert(parameter != parametersEnd);
    newEKFParams->ransacChi2Threshold = atof(parameter->second.c_str());

    parameter = parameters.find(CONFIG_EKF_INVDEPTH_LINEARITY_INDEX_THRESHOLD);
    assert(parameter != parametersEnd);
    newEKFParams->inverseDepthLinearityIndexThreshold = atof(parameter->second.c_str());

    return newEKFParams;
}

// ------------------------------------------------------------------------------------------
