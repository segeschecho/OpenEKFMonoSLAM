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

#ifndef __MODULES_1POINTRANSACEKF_MATCHING_H__
#define __MODULES_1POINTRANSACEKF_MATCHING_H__

#include "ImageFeaturePrediction.h"
#include "MapFeature.h"

class FeatureMatch
{
public:
    FeatureMatch() : featureIndex(-1), distance(-1) {};
    ~FeatureMatch() {};

    int featureIndex;
    double imagePos[2];
    cv::Mat imagePosDescriptor;
    float distance;
};

typedef std::vector<FeatureMatch *> VectorFeatureMatch;

/*
 * Para un conjunto de features predichos, busca los features en la imagen
 * que tienen un descriptor similar a cada prediccion correspondiente.
 *
 * Parametros:
 * image = imagen en la que se realizo la prediccion
 * features = vector de features del estado (todos los features)
 * vectorfeaturePrediction = vector de features predichos
 *
 * Devuelve:
 * matches = conjunto de matches. Cada match tiene un featureIndex y una posicion
 *           en la imagen donde se encontro un feature de similar descriptor.
 */

void matchPredictedFeatures( const cv::Mat& image,
                             const VectorMapFeature& features,
                             const VectorImageFeaturePrediction& vectorfeaturePrediction,
                             VectorFeatureMatch &matches );

#endif //__MODULES_1POINTRANSACEKF_MATCHING_H__
