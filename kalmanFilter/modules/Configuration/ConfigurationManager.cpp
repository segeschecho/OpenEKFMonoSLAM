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

#include "ConfigurationManager.h"
#include "ConfigurationDefines.h"
#include "ConfigurationDataReader/ExtendedKalmanFilterConfiguration/ExtendedKalmanFilterConfiguration.h"
#include "ConfigurationDataReader/CameraCalibrationConfiguration/CameraCalibrationConfiguration.h"
#include "ConfigurationDataReader/FeatureDetectorConfiguration/FeatureDetectorConfiguration.h"
#include "ConfigurationDataReader/DescriptorExtractorConfiguration/DescriptorExtractorConfiguration.h"

// ------------------------------------------------------------------------------------------

IMPLEMENT_SINGLETON_METHODS(ConfigurationManager);

// ------------------------------------------------------------------------------------------
// Constructor and Destructor Implementation
// ------------------------------------------------------------------------------------------

ConfigurationManager::ConfigurationManager()
{
    _ekfConfig = new ExtendedKalmanFilterConfiguration();
    _camCalibConfig = new CameraCalibrationConfiguration();
    _featureDetectorConfig = new FeatureDetectorConfiguration();
    _descriptorExtractorConfig = new DescriptorExtractorConfiguration();
    
    cameraCalibration = NULL;
    featureDetector = NULL;
    descriptorExtractor = NULL;
    ekfParams = NULL;
}

// ------------------------------------------------------------------------------------------

ConfigurationManager::~ConfigurationManager()
{
    delete _ekfConfig;
    delete _camCalibConfig;
    delete _featureDetectorConfig;
    delete _descriptorExtractorConfig;
}

// ------------------------------------------------------------------------------------------
// Methods Implementation
// ------------------------------------------------------------------------------------------

bool ConfigurationManager::loadConfigurationFromFile( const char *fileName )
{
    cv::FileStorage fileStorage(fileName, cv::FileStorage::READ);

    if (!fileStorage.isOpened())
    {
        std::cerr << "File " << fileName << " could not be opened to load configuration." << std::endl;
        return false;
    }

    std::string value;
    void* configObject;
    const cv::FileNode &runConfigNode = fileStorage[CONFIG_RUN_CONFIG_KEY];
    
    runConfigNode[CONFIG_RUN_CONFIG_EKF_KEY] >> value;
    configObject = _ekfConfig->loadNodeFromFileNode(fileStorage[CONFIG_EKF_KEY], value.c_str());
    if( !configObject ) return false;
    ekfParams = static_cast<ExtendedKalmanFilterParameters *>(configObject);

    runConfigNode[CONFIG_RUN_CONFIG_CAMERA_CALIB_KEY] >> value;
    configObject = _camCalibConfig->loadNodeFromFileNode(fileStorage[CONFIG_CAMERA_CALIB_KEY], value.c_str());
    if( !configObject ) return false;
    cameraCalibration = static_cast<CameraCalibration *>(configObject);

    runConfigNode[CONFIG_RUN_CONFIG_FEATURE_DETECTOR_KEY] >> value;
    configObject = _featureDetectorConfig->loadNodeFromFileNode(fileStorage[CONFIG_FEATURE_DETECTOR_KEY], value.c_str());
    if( !configObject ) return false;
    featureDetector = static_cast<cv::FeatureDetector *>(configObject);

    runConfigNode[CONFIG_RUN_CONFIG_DESCRIPTOR_EXTRACTOR_KEY] >> value;
    configObject = _descriptorExtractorConfig->loadNodeFromFileNode(fileStorage[CONFIG_DESCRIPTOR_EXTRACTOR_KEY], value.c_str());
    if( !configObject ) return false;
    descriptorExtractor = static_cast<cv::DescriptorExtractor *>(configObject);

    fileStorage.release();

    return true;
}

// ------------------------------------------------------------------------------------------
