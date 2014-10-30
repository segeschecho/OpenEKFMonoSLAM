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

#ifndef __MODULES_1POINTRANSACEKF_MEASUREMENT_PREDICTION_H___
#define __MODULES_1POINTRANSACEKF_MEASUREMENT_PREDICTION_H___

#include "../Core/Base.h"
#include "ImageFeaturePrediction.h"
#include "MapFeature.h"

class State;

void predictMeasurementState( const State &state, const VectorMapFeature &features, const std::vector<int> &featureIndexes,
                             VectorImageFeaturePrediction &predictedFeatures, VectorMapFeature &notPredictedFeatures );

/*
 * Realiza la prediccion de las mediciones
 * Parametros:
 *                    state: Es el estado de la camara, se corresponde al X_t_t-1 (luego de la prediccion del estado)
 * predictedStateCovariance: Es la matriz de covarianza del estado que se corresponde con al P_t_t-1 (luego de la prediccion)
 *                 features: Los features del mapa que se desean predecir
 *           featureIndexes: Los indices correspondientes a los features. Este parametro es necesario pues los features en si
 *                           no mantienen su featureIndex. Puede ser vacio, lo que querria decir que features deberia ser todos los
 *                           del mapa, y se hara la prediccion sobre todos ellos.
 *
 * Devuelve:
 * predictedDistortedFeatures: Es la posicion en la imagen de todos los features segun la prediccion del kalman (media de la gaussiana).
 *  predictedFeatureJacobians: Es el jacobiano de cada prediccion de feature. Es necesario mas adelante en el algoritmo.
 */

void predictCameraMeasurements( const State &state,
                                const Matd &predictedStateCovariance,
                                const VectorMapFeature &features,
                                const std::vector<int> &featureIndexes,
                                VectorImageFeaturePrediction &predictedDistortedFeatures,
                                VectorMatd &predictedFeatureJacobians,
                                VectorMapFeature &notPredictedFeatures);

#endif // __MODULES_1POINTRANSACEKF_MEASUREMENT_PREDICTION_H___
