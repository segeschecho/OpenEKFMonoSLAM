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

#include "../../ConfigurationDefines.h"

#include "DescriptorExtractorConfiguration.h"
#include "DescriptorExtractorFactory.h"
#include "DescriptorExtractorTypes.h"

// ------------------------------------------------------------------------------------------
// Constructor and Destructor Implementation
// ------------------------------------------------------------------------------------------

DescriptorExtractorConfiguration::DescriptorExtractorConfiguration() {}

// ------------------------------------------------------------------------------------------

DescriptorExtractorConfiguration::~DescriptorExtractorConfiguration()
{
    Dictionary< void* >::iterator itMap = _dataMap.begin();
    Dictionary< void* >::iterator itMapEnd = _dataMap.end();

    while (itMap != itMapEnd)
    {
        delete static_cast<cv::DescriptorExtractor *>(itMap->second);
        itMap++;
    }
}

// ------------------------------------------------------------------------------------------
// Inherited Methods Implementation
// ------------------------------------------------------------------------------------------

void* DescriptorExtractorConfiguration::loadNodeFromFileNode(const cv::FileNode &descriptorExtractorsFileNode, const char *alias)
{
    Dictionary<void *>::const_iterator it = _dataMap.find(alias);
    if (it != _dataMap.end())
    {
        std::cerr << "ERROR: Descriptor extractor configuration with name " << alias << " was already defined.";
        return it->second;
    }

    cv::FileNode descriptorExtractor = descriptorExtractorsFileNode[alias];

    cv::FileNodeIterator itParametersEnd = descriptorExtractor.end();
    cv::FileNodeIterator itParameters = descriptorExtractor.begin();

    if (itParameters == itParametersEnd)
    {
        std::cerr << "ERROR: Descriptor extractor configuration with name " << alias << " was not found.";
        return NULL;
    }

    if (strcmp((*itParameters).name().c_str(), CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_KEY) != 0)
    {
        std::cerr << "ERROR: Descriptor extractor configuration with name " << alias << " must have a type.";
        return NULL;
    }

    DESCRIPTOR_EXTRACTOR_TYPE newDescriptorExtractorType = descriptorExtractorTypeStringToType( ((std::string)*itParameters).c_str() );

    itParameters++;

    Dictionary<std::string> parameters;
    while (itParameters != itParametersEnd)
    {
        std::string sParameterName = (*itParameters).name();
        std::string parameterValue = (std::string)(*itParameters);

        parameters[sParameterName] = parameterValue;

        itParameters++;
    }

    DescriptorExtractorFactory &deFactoryInstance = DescriptorExtractorFactory::getInstance();
    cv::DescriptorExtractor *newDescriptorExtractor = deFactoryInstance.createDescriptorExtractor( newDescriptorExtractorType,
                                                                                                   alias,
                                                                                                   parameters );

    _dataMap[alias] = newDescriptorExtractor;

    return newDescriptorExtractor;
}

// ------------------------------------------------------------------------------------------
// Private Methods Implementation
// ------------------------------------------------------------------------------------------

inline DESCRIPTOR_EXTRACTOR_TYPE DescriptorExtractorConfiguration::descriptorExtractorTypeStringToType( const char *type ) const
{
    if (type == NULL)
    {
        return DESCRIPTOR_EXTRACTOR_TYPE_INVALID;
    }

    std::string sType(type);

#ifndef ANDROID
    if (sType.compare(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_KEY) == 0)
    {
        return DESCRIPTOR_EXTRACTOR_TYPE_SURF;
    }
    if (sType.compare(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_KEY) == 0)
    {
        return DESCRIPTOR_EXTRACTOR_TYPE_SIFT;
    }
#endif
    if (sType.compare(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_ORB_KEY) == 0)
    {
        return DESCRIPTOR_EXTRACTOR_TYPE_ORB;
    }
    if (sType.compare(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_BRIEF_KEY) == 0)
    {
        return DESCRIPTOR_EXTRACTOR_TYPE_BRIEF;
    }

    return DESCRIPTOR_EXTRACTOR_TYPE_INVALID;
}

// ------------------------------------------------------------------------------------------
