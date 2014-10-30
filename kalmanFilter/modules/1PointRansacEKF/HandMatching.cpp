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

#include "HandMatching.h"

static std::vector< VectorFeatureMatch > matchesVector;
static std::vector< VectorImageFeatureMeasurement > measurementVector;

void loadHandMatches(std::string filePath)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Couldn't open file " << filePath << " to load matches." << std::endl;
        return;
    }
    
    cv::FileNode rootFileNode = fs.root();
    cv::FileNodeIterator itEnd = rootFileNode.end();
    
    for (cv::FileNodeIterator it = rootFileNode.begin(); it != itEnd; it++)
    {
        const char *currFrameName = (*it).name().c_str();
        
        cv::FileNode currFrameFileNode = rootFileNode[currFrameName];
        VectorFeatureMatch currFrameMatches;
        VectorImageFeatureMeasurement currFrameMeasurements;
        
        size_t currFrameMatchesSize = currFrameFileNode.size();
        for (int i = 0; i < currFrameMatchesSize; ++i)
        {
            cv::FileNode currMatchesFileNode = currFrameFileNode[i];
            
            int currMatchX = currMatchesFileNode["x"];
            int currMatchY = currMatchesFileNode["y"];
            
            if (currMatchX > 0 && currMatchY > 0)
            {
                FeatureMatch *currMatch = new FeatureMatch;
                
                currMatch->featureIndex = i;
                currMatch->imagePos[0] = currMatchX;
                currMatch->imagePos[1] = currMatchY;
                
                currFrameMatches.push_back(currMatch);
                
                ImageFeatureMeasurement *measurement = new ImageFeatureMeasurement;
                measurement->imagePos[0] = currMatchX;
                measurement->imagePos[1] = currMatchY;
                
                currFrameMeasurements.push_back(measurement);
            }
        }
        
        matchesVector.push_back(currFrameMatches);
        measurementVector.push_back(currFrameMeasurements);
    }

    fs.release();
}

void handmatchMeasurementsForStep(int step, VectorImageFeatureMeasurement &measurements)
{
    measurements = measurementVector[step];
    
}

void handmatchForStep(int step, VectorFeatureMatch &matches)
{
    matches = matchesVector[step];
}