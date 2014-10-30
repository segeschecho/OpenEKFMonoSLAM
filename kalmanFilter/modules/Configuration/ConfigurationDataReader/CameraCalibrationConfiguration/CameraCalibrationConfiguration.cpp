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
#include "CameraCalibrationConfiguration.h"
#include "CameraCalibration.h"

// ------------------------------------------------------------------------------------------
// Constructor and Destructor Implementation
// ------------------------------------------------------------------------------------------

CameraCalibrationConfiguration::CameraCalibrationConfiguration()
{
}

// ------------------------------------------------------------------------------------------

CameraCalibrationConfiguration::~CameraCalibrationConfiguration()
{
    Dictionary< void* >::iterator itMap = _dataMap.begin();
    Dictionary< void* >::iterator itMapEnd = _dataMap.end();
    
    while (itMap != itMapEnd)
    {
        delete static_cast<CameraCalibration *>(itMap->second);
        itMap++;
    }
}

// ------------------------------------------------------------------------------------------
// Inherited Methods Implementation
// ------------------------------------------------------------------------------------------

void* CameraCalibrationConfiguration::loadNodeFromFileNode(const cv::FileNode &camerasCalibrationFileNode, const char *alias)
{
    Dictionary<void *>::const_iterator it = _dataMap.find(alias);
    if (it != _dataMap.end())
    {
        std::cerr << "ERROR: Camera calibration configuration with name " << alias << " was already defined.";
        return it->second;
    }

    cv::FileNode cameraCalibrationNode = camerasCalibrationFileNode[alias];
    
    cv::FileNodeIterator itParametersEnd = cameraCalibrationNode.end();
    cv::FileNodeIterator itParameters = cameraCalibrationNode.begin();
    
    if (itParameters == itParametersEnd)
    {
        std::cerr << "ERROR: Camera calibration configuration with name " << alias << " was not found.";
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
    
    CameraCalibration* newCameraCalibration = createNewCameraCalibration(alias, parameters);
    _dataMap[alias] = newCameraCalibration;
    
    return newCameraCalibration;
}

// ------------------------------------------------------------------------------------------
// Private Methods Implementation
// ------------------------------------------------------------------------------------------

CameraCalibration* CameraCalibrationConfiguration::createNewCameraCalibration( std::string cameraName,
                                                                               const Dictionary<std::string> &parameters ) const
{
    CameraCalibration *newCameraCalibration = new CameraCalibration();
    Dictionary<std::string>::const_iterator parameter;
    
    newCameraCalibration->cameraName = cameraName;
    
    parameter = parameters.find(CONFIG_CAMERA_PIXELS_X_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->pixelsX = atoi(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_PIXELS_Y_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->pixelsY = atoi(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_FX_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->fx = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_FY_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->fy = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_K1_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->k1 = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_K2_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->k2 = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_CX_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->cx = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_CY_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->cy = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_DX_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->dx = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_INTRINSIC_DY_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->dy = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_PIXEL_ERROR_X_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->pixelErrorX = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_PIXEL_ERROR_Y_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->pixelErrorY = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_ANGULAR_VISION_X_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->angularVisionX = atof(parameter->second.c_str());
    
    parameter = parameters.find(CONFIG_CAMERA_ANGULAR_VISION_Y_KEY);
    assert(parameter != parameters.end());
    newCameraCalibration->angularVisionY = atof(parameter->second.c_str());

    return newCameraCalibration;
}

// ------------------------------------------------------------------------------------------