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

#include <ctype.h>

#include "FeatureDetectorConfiguration.h"

#include "../../ConfigurationDefines.h"
#include "FeatureDetectorFactory.h"

// ------------------------------------------------------------------------------------------
// Constructor and Destructor Implementation
// ------------------------------------------------------------------------------------------

FeatureDetectorConfiguration::FeatureDetectorConfiguration() {}

// ------------------------------------------------------------------------------------------

FeatureDetectorConfiguration::~FeatureDetectorConfiguration()
{
    Dictionary< void* >::iterator itMap = _dataMap.begin();
    Dictionary< void* >::iterator itMapEnd = _dataMap.end();

    while (itMap != itMapEnd)
    {
        delete static_cast<cv::FeatureDetector *>(itMap->second);
        itMap++;
    }
}

// ------------------------------------------------------------------------------------------
// Inherited Methods Implementation
// ------------------------------------------------------------------------------------------

void* FeatureDetectorConfiguration::loadNodeFromFileNode(const cv::FileNode &featureDetectorsFileNode, const char *alias)
{
    Dictionary<void *>::const_iterator it = _dataMap.find(alias);
    if (it != _dataMap.end())
    {
        std::cerr << "ERROR: Feature detector configuration with name " << alias << " was already defined.";
        return it->second;
    }

    cv::FileNode featureDetector = featureDetectorsFileNode[alias];

    cv::FileNodeIterator itParametersEnd = featureDetector.end();
    cv::FileNodeIterator itParameters = featureDetector.begin();

    if (itParameters == itParametersEnd)
    {
        std::cerr << "ERROR: Feature detector configuration with name " << alias << " was not found.";
        return NULL;
    }

    if (strcmp((*itParameters).name().c_str(), CONFIG_FEATURE_DETECTOR_TYPE_KEY) != 0)
    {
        std::cerr << "ERROR: Feature detector configuration with name " << alias << " must have a type.";
        return NULL;
    }

    FEATURE_DETECTOR_TYPE newFeatureDetectorType = featureDetectorTypeStringToType( ((std::string)*itParameters).c_str() );

    itParameters++;

    Dictionary<std::string> parameters;
    while (itParameters != itParametersEnd)
    {
        std::string sParameterName = (*itParameters).name();
        std::string parameterValue = (std::string)(*itParameters);

        parameters[sParameterName] = parameterValue;

        itParameters++;
    }

    FeatureDetectorFactory &fdFactoryInstance = FeatureDetectorFactory::getInstance();
    cv::FeatureDetector *newFeatureDetector = fdFactoryInstance.createFeatureDetector( newFeatureDetectorType,
                                                                                       alias,
                                                                                       parameters );

    _dataMap[alias] = newFeatureDetector;

    return newFeatureDetector;
}

// ------------------------------------------------------------------------------------------
// Private Methods Implementation
// ------------------------------------------------------------------------------------------

inline FEATURE_DETECTOR_TYPE FeatureDetectorConfiguration::featureDetectorTypeStringToType( const char *type ) const
{
    if (type == NULL)
    {
        return FEATURE_DETECTOR_TYPE_INVALID;
    }

    std::string sType(type);

#ifndef ANDROID
    if (sType.compare(CONFIG_FEATURE_DETECTOR_TYPE_SURF_KEY) == 0)
    {
        return FEATURE_DETECTOR_TYPE_SURF;
    }
    if (sType.compare(CONFIG_FEATURE_DETECTOR_TYPE_SIFT_KEY) == 0)
    {
        return FEATURE_DETECTOR_TYPE_SIFT;
    }
#endif
    if (sType.compare(CONFIG_FEATURE_DETECTOR_TYPE_FAST_KEY) == 0)
    {
        return FEATURE_DETECTOR_TYPE_FAST;
    }
    if (sType.compare(CONFIG_FEATURE_DETECTOR_TYPE_ORB_KEY) == 0)
    {
        return FEATURE_DETECTOR_TYPE_ORB;
    }
    if (sType.compare(CONFIG_FEATURE_DETECTOR_TYPE_STAR_KEY) == 0)
    {
        return FEATURE_DETECTOR_TYPE_STAR;
    }

    return FEATURE_DETECTOR_TYPE_INVALID;
}

// ------------------------------------------------------------------------------------------
