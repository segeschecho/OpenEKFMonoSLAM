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

#include <list>

#include "../Gui/Draw.h"

#include "../Core/EKFMath.h"
#include "../Core/Base.h"

#include "../Configuration/ConfigurationManager.h"

#include "State.h"

#include "Matching.h"

static cv::Scalar ellipseColorWhite(255, 255, 255);

double computeDistance( const cv::Mat &descriptor,
                       const cv::Mat &candidateDescriptors,
                       uint candidateDescriptorIndex )
{
    int type = descriptor.type();

    assert((type == candidateDescriptors.type()) && (type == CV_32F || type == CV_8U));
    assert(descriptor.cols == candidateDescriptors.cols);

    double distance = 0.0f;

    // NormL2
    if (type == CV_32F)
    {
        const float *currDescriptor = &candidateDescriptors.ptr<float>()[candidateDescriptorIndex * candidateDescriptors.cols];
        const float *descriptorPtr = descriptor.ptr<float>();

        for (int j = 0; j < descriptor.cols; ++j)
        {
            float subs = descriptorPtr[j] - currDescriptor[j];
            distance += subs * subs;
        }

        distance = sqrt(distance);
    }

    // Hamming
    if (type == CV_8U)
    {
        const uchar *currDescriptor = &candidateDescriptors.ptr<uchar>()[candidateDescriptorIndex * candidateDescriptors.cols];
        const uchar *descriptorPtr = descriptor.ptr<uchar>();
        uint distanceInt = 0;

        for (int j = 0; j < descriptor.cols; ++j)
        {
            uchar x = descriptorPtr[j];
            uchar y = currDescriptor[j];
            uchar xorVal = x ^ y;

            distanceInt += popCountTable[xorVal];
        }

        distance = distanceInt;
    }

    return distance;
}

// ------------------------------------------------------------------------------------------------------------
// Funcion que busca matchear un descriptor con exactamente uno del conjunto de candidatos
//
// Parametros:
//      descriptor                  = aquel que se quiere matchear contra los candidatos.
//
//      candidateDescriptors        = conjunto de descriptores candidato. Se espera que el primer valor de
//                                    cada descriptor de este conjunto esten en las direcciones de memoria
//                                    descriptorSize * i, con 0 <= i < candidateDescriptorsCount.
//
//      descriptorSize              = longitud de los descriptores en bits.
//
//      descriptorsCount            = longitud del arreglo candidateDescriptors / descriptorSize.
//
//      mask                        = mask[i] = false significa que el i-esimo descriptor
//                                    no se tiene en cuenta para el matcheo
//
//  Retorno:
//      MatchIndexAndDistance es un par <int, double>, donde el primero es el indice del candidato
//      matcheado de candidateDescriptors, y el segundo es la distancia (norma l2).

void findBestNMatches( uint nBest,
                       const cv::Mat &descriptor,
                       const cv::Mat &candidateDescriptors,
                       const uchar *mask,
                       std::list<cv::DMatch> &bestNMatches )
{
    bestNMatches.clear();

    double minDistance = -1.0L;

    for (int i = 0; i < candidateDescriptors.rows; ++i)
    {
        if (!mask || (mask && mask[i]))
        {
            double distance = computeDistance(descriptor, candidateDescriptors, i);

            if (distance < minDistance || bestNMatches.size() < 2)
            {
                minDistance = minDistance < 0 ? distance : MIN(minDistance, distance);
                bestNMatches.push_front(cv::DMatch(0, i, static_cast<float>(distance)));

                if (bestNMatches.size() > nBest)
                {
                    bestNMatches.pop_back();
                }
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------------------

void matchICDescriptors( const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors,
                         std::vector<cv::DMatch>& matches, const cv::Mat& mask )
{
    matches.clear();

    const uchar *maskPtr = mask.ptr<uchar>();

    for (int i = 0; i < queryDescriptors.rows; ++i)
    {
        const uchar *descriptorMask = &maskPtr[i * trainDescriptors.cols];
        const cv::Mat& queryDescriptor = queryDescriptors.row(i);

        std::list<cv::DMatch> bestNMatches;
        findBestNMatches( 2,
                          queryDescriptor,
                          trainDescriptors,
                          descriptorMask,
                          bestNMatches );

        // Si solamente encontre un unico match (lo que querria decir que solamente tenia un unico descriptor en train) devuelvo ese,
        // Si encontre 2 mejores matches, verifico que la 1er mejor distancia multiplicada por un factor sea menor a la 2da mejor distancia
        if ( bestNMatches.size() == 1 || (bestNMatches.size() >= 2 &&
                                          bestNMatches.front().distance <=
                                             bestNMatches.back().distance *
                                             ConfigurationManager::getInstance().ekfParams->matchingCompCoefSecondBestVSFirst) )
        {
            matches.push_back(bestNMatches.front());
        }
    }
}

// ------------------------------------------------------------------------------------------------------------

void matchPredictedFeatures( const cv::Mat& image,
                             const VectorMapFeature& features,
                             const VectorImageFeaturePrediction& vectorfeaturePrediction,
                             VectorFeatureMatch &matches )
{
    ConfigurationManager &configManager = ConfigurationManager::getInstance();

    cv::Mat imageMask( cv::Mat::zeros(image.rows, image.cols, CV_8UC1) );

    size_t vectorfeaturePredictionSize = vectorfeaturePrediction.size();

    // armo la mascara, para evitar buscar features fuera de los elipses de los features predichos
    for (int i = 0; i < vectorfeaturePredictionSize; ++i)
    {
        ImageFeaturePrediction *featurePrediction = vectorfeaturePrediction[i];
        drawUncertaintyEllipse2D( imageMask,
                                  cv::Point2d(featurePrediction->imagePos[0], featurePrediction->imagePos[1]),
                                  featurePrediction->covarianceMatrix,
                                  2.0L*MAX(configManager.cameraCalibration->pixelsX, configManager.cameraCalibration->pixelsY),
                                  ellipseColorWhite,
                                  true );
    }

    // detectamos features
    std::vector<cv::KeyPoint> imageKeypoints;
    configManager.featureDetector->detect(image, imageKeypoints, imageMask);

    // extraemos descriptores
    cv::Mat descriptors;
    configManager.descriptorExtractor->compute(image, imageKeypoints, descriptors);

    // IMPORTANTE: el size del imageKeypoints se DEBE tomar despues de extraer descriptores
    // ya que este metodo puede eliminar keypoints sobre los que no se haya podido computar
    // los descriptores.
    size_t imageKeypointsSize = imageKeypoints.size();

    for (int i = 0; i < vectorfeaturePredictionSize; ++i)
    {
        cv::Mat mask(cv::Mat::zeros(1, descriptors.rows, CV_8UC1));

        ImageFeaturePrediction *featurePrediction = vectorfeaturePrediction[i];

        MapFeature *currMapFeature= features[featurePrediction->featureIndex];

        cv::Size2f axesSize(0.0f, 0.0f);
        double angle = 0.0L;
        matrix2x2ToUncertaintyEllipse2D(featurePrediction->covarianceMatrix, axesSize, angle);

        for (int j = 0; j < imageKeypointsSize; ++j)
        {
            // Si el feature cae dentro de la elipse, agregarlo como posible candidato
            if ( pointIsInsideEllipse( imageKeypoints[j].pt,
                                       cv::Point2d(featurePrediction->imagePos[0], featurePrediction->imagePos[1]),
                                       axesSize,
                                       angle ) )
            {
                mask.at<uchar>(0, j) = true;
            }
        }

        std::vector<cv::DMatch> currFeatureMatch;
        matchICDescriptors( currMapFeature->descriptor,
                            descriptors,
                            currFeatureMatch,
                            mask );

        // Si encontramos algun match lo agregamos al vector de resultado
        if (currFeatureMatch.size() > 0)
        {
            cv::KeyPoint& matchedKeypoint = imageKeypoints[currFeatureMatch[0].trainIdx];

            // Agregamos un match al resultado
            FeatureMatch *newMatch = new FeatureMatch;
            newMatch->featureIndex = featurePrediction->featureIndex;
            newMatch->imagePos[0] = static_cast<double>(matchedKeypoint.pt.x);
            newMatch->imagePos[1] = static_cast<double>(matchedKeypoint.pt.y);
            newMatch->distance = currFeatureMatch[0].distance;

            descriptors.row(currFeatureMatch[0].trainIdx).copyTo(newMatch->imagePosDescriptor);

            matches.push_back(newMatch);
        }
    }
}
