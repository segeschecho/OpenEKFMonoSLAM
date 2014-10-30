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

#include "../Configuration/ConfigurationManager.h"

#include "1PointRansac.h"
#include "Update.h"
#include "State.h"
#include "MeasurementPrediction.h"

#ifdef DEBUG_SHOW_RANSAC_INFO
#include "../Gui/Draw.h"
#endif

// --------------------------------------------------------------------------------------------------------------------
// Devuelve los indices (posiciones del arreglo de matches) de los matches que estan por debajo de un threshold.
// Se devuelven estos indices, para aprovechar que los matches, las predicciones y los jacobianos estan en arreglos
// con el mismo orden
// --------------------------------------------------------------------------------------------------------------------
void matchesBelowAThreshold(const VectorFeatureMatch &matches,
                            const VectorImageFeaturePrediction &predictedImageFeatures,
                            double threshold,
                            std::vector<int> &supportIndexesInMatchesVector)
{
    // Para cada feature predicho se busca su match y se calcula la distancia
    for (uint i = 0; i < predictedImageFeatures.size(); ++i)
    {
        ImageFeaturePrediction *prediction = predictedImageFeatures[i];

        bool wasFound = false;
        int matchIndex = 0;
        while (!wasFound && matchIndex < matches.size())
        {
            FeatureMatch *match = matches[matchIndex];

            // Se calcula la distancia y se cuenta si es buen feature.
            if (match->featureIndex == prediction->featureIndex)
            {
                double xDist = match->imagePos[0] - prediction->imagePos[0];
                double yDist = match->imagePos[1] - prediction->imagePos[1];
                double dist = sqrt(xDist*xDist + yDist*yDist);

                if (dist < threshold)
                {
                    // Si es un buen match se guarda el indice en el arreglo de matches para luego
                    // obtener la prediccion y los jacobianos asociados.(que se encuentran en el mismo orden que el arreglo matches)
                    supportIndexesInMatchesVector.push_back(matchIndex);
                }

                wasFound = true;
            }

            matchIndex++;
        }
    }
}

// --------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------
void selectRandomMatch(const VectorFeatureMatch &matches, uint index, FeatureMatch *&match)
{
    // TODO: Hacer random, ahora se hace directo para que sea reproducible.
    match = matches[index];
}

// --------------------------------------------------------------------------------------------------------------------
// Realiza el ciclo de ransac, donde se obtienen los matches considerados inliers.
// Esta informacion se devuelve, para por fuera, rescatar outliers y poder hacer una mejor estimacion
// En inlierJacobianIndexes se guardan los indices(posicion en el arreglo) de las predicciones,
// ya que se corresponden con la posicion del los jacobianos asociados y
// que son necesarios para hacer el update.
// --------------------------------------------------------------------------------------------------------------------
void ransac( const State &state, Matd &covariance,
             const VectorImageFeaturePrediction &predictedMatchedFeatures,
             const VectorMatd &predictedMatchedJacobians,
             const VectorFeatureMatch &matches,
             VectorFeatureMatch &inlierMatches,
             VectorImageFeaturePrediction &inlierPredictions,
             VectorMatd &inlierJacobians,
             VectorFeatureMatch &outlierMatches )
{
    size_t matchesSize = matches.size();
    if (matches.size() == 0)
    {
        return;
    }

    uint numberOfHipotesis = 1000;

    ExtendedKalmanFilterParameters *ekfParams = ConfigurationManager::getInstance().ekfParams;

    // threshold para buscar los matches 2 * sigma_pixels
    double threshold = ekfParams->ransacThresholdPredictDistance;

    std::vector<int> inlierIndexesInMatchesVector;

    for (uint i = 0; i < numberOfHipotesis && i < matchesSize; ++i)
    {
        // seleccionar matches random
        FeatureMatch *match = NULL;
        selectRandomMatch(matches, i, match);

        // Se toma la prediccion correspondiente el matching
        ImageFeaturePrediction *predictedMatchedFeature = predictedMatchedFeatures[i];

        // Se genera un arreglo para el feature predicho.
        VectorImageFeaturePrediction prediction;
        prediction.push_back(predictedMatchedFeature);

        VectorFeatureMatch matching;
        matching.push_back(match);

        // Se buscan los 2 jacobianos del feature
        VectorMatd jacobians;
        jacobians.push_back(predictedMatchedJacobians[i]);

        // Se hace un update parcial (solo para el estado)
        // se copia el estado actual
        State temporalState(state);

        updateOnlyState(prediction, matching, jacobians, temporalState, covariance);

        // Se predicen TODAS las mediciones con el estado actualizado
        VectorImageFeaturePrediction predictedImageFeatures;
        VectorMapFeature unseenFeatures;
        std::vector<int> mapFeatureIndexes;
        // se pasa mapFeatureIndexes vacio para que sepa que son para todos los features.
        predictMeasurementState(temporalState, temporalState.mapFeatures, mapFeatureIndexes, predictedImageFeatures, unseenFeatures);

        // Encontrar matches debajo de un threshold
        std::vector<int> supportIndexesInMatchesVector;

        matchesBelowAThreshold(matches, predictedImageFeatures, threshold, supportIndexesInMatchesVector);

        // Si la cantidad de matches encontrado es mayor que la que se tiene actualmente, actualizar.
        if (supportIndexesInMatchesVector.size() > inlierIndexesInMatchesVector.size())
        {
#ifdef DEBUG_SHOW_RANSAC_INFO
            std::string windowName = "Inliers en el ciclo de ransac, prediccion hecha en base a un solo match";
            showRansacInliers(matches, predictedImageFeatures, supportIndexesInMatchesVector, match, windowName);
#endif
            // Actualizo el conjunto de inliers
            inlierIndexesInMatchesVector = supportIndexesInMatchesVector;

            // Se actualiza la cantidad de iteraciones
            // 1 - w
            double e = 1.0L - (double)inlierIndexesInMatchesVector.size()/(double)matches.size();

            numberOfHipotesis = static_cast<int>( log(1.0L - ekfParams->ransacAllInliersProbability)/log(1.0L - (1.0L - e)) );
        }
        
        // Libero la memoria auxiliar
        size_t predictedImageFeaturesSize = predictedImageFeatures.size();
        for (uint j = 0; j < predictedImageFeaturesSize; ++j)
        {
            delete predictedImageFeatures[j];
        }
    }

    // Se devuelven los matches, predicciones y jacobianos de los indices (del vector matches) inliers
    inlierMatches.clear();
    inlierPredictions.clear();
    inlierJacobians.clear();
    outlierMatches.clear();

    // Se reserva lugar para que los push_back sean en O(1)
    inlierMatches.reserve(inlierIndexesInMatchesVector.size());
    inlierPredictions.reserve(inlierIndexesInMatchesVector.size());
    inlierJacobians.reserve(inlierIndexesInMatchesVector.size());
    outlierMatches.reserve(matches.size() - inlierIndexesInMatchesVector.size());

    // Se arma una mascara para identificar los lugares de matches donde estan los inliers
    std::vector<bool> inlierMatchesMask(matches.size(), false);

    // FIXME: Esto se puede hacer sin la mascara usando la imformacion de que
    // los inliers mantienen el orden relativo que tienen en el vector "matches"
    for (uint i = 0; i < inlierIndexesInMatchesVector.size(); ++i)
    {
        int index = inlierIndexesInMatchesVector[i];

        inlierMatchesMask[index] = true;
    }

    // Con la mascara se separan los matches inliers y los outliers
    for (uint i = 0; i < inlierMatchesMask.size(); ++i)
    {
        bool is_inlier = inlierMatchesMask[i];

        if (is_inlier)
        {
            inlierMatches.push_back(matches[i]);
            inlierPredictions.push_back(predictedMatchedFeatures[i]);
            inlierJacobians.push_back(predictedMatchedJacobians[i]);
        }
        else
        {
            outlierMatches.push_back(matches[i]);
        }
    }

#ifdef DEBUG
    std::cout << "cantidad de matches: " << matches.size() << std::endl;
    std::cout << "cantidad de inliers: " << inlierMatches.size() << std::endl;
    std::cout << "cantidad de ouliers: " << outlierMatches.size() << std::endl;
#endif
}
