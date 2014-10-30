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
#include "DescriptorExtractorFactory.h"

IMPLEMENT_SINGLETON_METHODS(DescriptorExtractorFactory);

// ------------------------------------------------------------------------------------------
// Constructor and Destructor Implementation
// ------------------------------------------------------------------------------------------

DescriptorExtractorFactory::~DescriptorExtractorFactory() {}

// ------------------------------------------------------------------------------------------

DescriptorExtractorFactory::DescriptorExtractorFactory() {}

// ------------------------------------------------------------------------------------------
// Methods implementation
// ------------------------------------------------------------------------------------------

cv::DescriptorExtractor *DescriptorExtractorFactory::createDescriptorExtractor( DESCRIPTOR_EXTRACTOR_TYPE type,
                                                                                std::string alias,
                                                                                const Dictionary<std::string> &parameters ) const
{
    cv::DescriptorExtractor *newDescriptorExtractor = NULL;

    switch (type)
    {
#ifndef ANDROID
        case DESCRIPTOR_EXTRACTOR_TYPE_SURF:
        {
            Dictionary<std::string>::const_iterator parametersEnd = parameters.end();
            Dictionary<std::string>::const_iterator parameter;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_HESSIAN_TRESHOLD_KEY);
            double hessianThreshold = (parameter != parametersEnd) ? atof(parameter->second.c_str()) : 400;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_OCTAVES_KEY);
            int nOctaves = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 4;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_OCTAVE_LAYERS_KEY);
            int nOctaveLayers = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 2;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_EXTENDED_KEY);
            bool extended = (parameter != parametersEnd) ? (parameter->second.compare( std::string(CONFIG_TRUE_KEY) ) == 0) : true;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SURF_UPRIGHT_KEY);
            bool upright =  (parameter != parametersEnd) ? (parameter->second.compare( std::string(CONFIG_TRUE_KEY) ) == 0) : false;

            newDescriptorExtractor = new cv::SURF(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
        }
        break;

        case DESCRIPTOR_EXTRACTOR_TYPE_SIFT:
        {
            Dictionary<std::string>::const_iterator parametersEnd = parameters.end();
            Dictionary<std::string>::const_iterator parameter;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_FEATURES_KEY);
            int nFeatures = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 0;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_OCTAVE_LAYERS_KEY);
            int nOctavesLayers = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 3;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_CONTRAST_THRESHOLD_KEY);
            double contrastThreshold = (parameter != parametersEnd) ? atof(parameter->second.c_str()) : 0.04L;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_EDGE_THRESHOLD_KEY);
            double edgeThreshold = (parameter != parametersEnd) ? atof(parameter->second.c_str()) : 10L;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_SIFT_SIGMA_KEY);
            double sigma = (parameter != parametersEnd) ? atof(parameter->second.c_str()) : 1.6L;

            newDescriptorExtractor = new cv::SIFT(nFeatures, nOctavesLayers, contrastThreshold, edgeThreshold, sigma);
        }
        break;
#endif
        case DESCRIPTOR_EXTRACTOR_TYPE_ORB:
        {
            newDescriptorExtractor = new cv::ORB();
        }
        break;

        case DESCRIPTOR_EXTRACTOR_TYPE_BRIEF:
        {
            Dictionary<std::string>::const_iterator parametersEnd = parameters.end();
            Dictionary<std::string>::const_iterator parameter;

            parameter = parameters.find(CONFIG_DESCRIPTOR_EXTRACTOR_TYPE_BRIEF_BYTES_LENGTH_KEY);
            int bytes = (parameter != parametersEnd) ? atoi(parameter->second.c_str()) : 32;

            newDescriptorExtractor = new cv::BriefDescriptorExtractor(bytes);
        }
        break;

        default:
        {
            std::cerr << "ERROR! Could not create descriptor extractor. Invalid descriptor extractor type." << std::endl;
        }
        break;
    }

    return newDescriptorExtractor;
}

// ------------------------------------------------------------------------------------------
