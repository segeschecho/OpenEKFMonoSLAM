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

#ifndef __MODULES_CONFIGURATION_EXTENDEDKALMANFILTERCONFIGURATION_EXTENDEDKALMANFILTERPARAMETERS_H__
#define __MODULES_CONFIGURATION_EXTENDEDKALMANFILTERCONFIGURATION_EXTENDEDKALMANFILTERPARAMETERS_H__

#include "../../../Core/Base.h"

class ExtendedKalmanFilterParameters
{
    public:
        // Constructor and Destructor
        ExtendedKalmanFilterParameters(){};
        ~ExtendedKalmanFilterParameters(){};

        std::string ekfParametersName;
    
        bool alwaysRemoveUnseenMapFeatures;
    
        uint maxMapFeaturesCount;
        uint maxMapSize;

        uint mapManagementFrequency;
        uint detectNewFeaturesImageAreasDivideTimes;
        uint minMatchesPerImage;

        uint reserveFeaturesDepth;
        uint reserveFeaturesInvDepth;
    
        double initInvDepthRho;
        double initLinearAccelSD;
        double initAngularAccelSD;
        
        double linearAccelSD;
        double angularAccelSD;
        double inverseDepthRhoSD;
    
        double detectNewFeaturesImageMaskEllipseSize;
    
        double matchingCompCoefSecondBestVSFirst;
        double goodFeatureMatchingPercent;
    
        double ransacThresholdPredictDistance;
        double ransacAllInliersProbability;
        double ransacChi2Threshold;
    
        double inverseDepthLinearityIndexThreshold;
};

//// Show function
//std::ostream& operator<<( std::ostream& os, const ExtendedKalmanFilterParameters& ekfParams)
//{
//    os << std::setprecision(20)
//       << ekfParams.ekfParametersName << std::endl
//       << "ReserveFeaturesDepth = " << ekfParams.reserveFeaturesDepth << std::endl
//       << "ReserveFeaturesInvDepth = " << ekfParams.reserveFeaturesInvDepth << std::endl
//       << "InitInvDepthRho = " << ekfParams.initInvDepthRho << std::endl
//       << "InitLinearAccelSD = " << ekfParams.initLinearAccelSD << std::endl
//       << "InitAngularAccelSD = " << ekfParams.initAngularAccelSD << std::endl
//       << "LinearAccelSD = " << ekfParams.linearAccelSD << std::endl
//       << "AngularAccelSD = " << ekfParams.angularAccelSD << std::endl
//       << "InverseDepthRhoSD = " << ekfParams.inverseDepthRhoSD << std::endl
//       << "MapManagementFrequency = " << ekfParams.mapManagementFrequency << std::endl
//       << "DetectNewFeaturesImageAreasDivideTimes = " << ekfParams.detectNewFeaturesImageAreasDivideTimes << std::endl
//       << "MatchingCompCoefSecondBestVSFirst = " << ekfParams.matchingCompCoefSecondBestVSFirst << std::endl
//       << "MinMatchesPerImage = " << ekfParams.minMatchesPerImage << std::endl
//       << "GoodFeatureMatchingPercent = " << ekfParams.goodFeatureMatchingPercent << std::endl
//       << "RansacThresholdPredictDistance = " << ekfParams.ransacThresholdPredictDistance << std::endl
//       << "RansacAllInliersProbability = " << ekfParams.ransacAllInliersProbability << std::endl
//       << "RansacChi2Threshold = " << ekfParams.ransacChi2Threshold << std::endl;
//
//    return os;
//}

#endif // __MODULES_CONFIGURATION_EXTENDEDKALMANFILTERCONFIGURATION_EXTENDEDKALMANFILTERPARAMETERS_H__
