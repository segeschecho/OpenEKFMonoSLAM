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

#include <algorithm>
#include <time.h>
#include <list>

#include "../Configuration/ConfigurationManager.h"
#include "../Core/EKFMath.h"

#include "../Gui/Draw.h"

#include "DetectNewImageFeatures.h"

#ifdef DEBUG_SHOW_NEW_FEATURES
#include <opencv/highgui.h>
#include "../Gui/Draw.h"
#endif

// ZoneInfo es una estructura auxiliar que guarda toda la informacion sobre
// una zona de la imagen necesaria para ejecutar la heuristica de busqueda de nuevos features
struct ZoneInfo
{
    // Predicciones que caen en la zona (su media)
    VectorImageFeaturePrediction featuresPrediction;

    // Nuevos features encontrados, candidatos a ser agregados
    VectorImageFeatureMeasurement candidateImageFeatureMeasurement;

    // Nuevos features encontrados y agregados
    VectorImageFeatureMeasurement imageFeatureMeasurementAdded;

    // Redundancia para evitar pedir size de los vectores anteriores.
    int candidateImageFeatureMeasurementLeft;

    // Redundancia para evitar pedir size de los vectores anteriores.
    int predictionsPlusFeaturesCount;

    // Id de la zona que contiene esta info
    int zoneId;
};

// ------------------------------------------------------------------------------------------------------------

int sortCompare( const void * elem1, const void * elem2 )
{
    const ZoneInfo *zone1 = *(ZoneInfo **)elem1;
    const ZoneInfo *zone2 = *(ZoneInfo **)elem2;

    int zone1Value = zone1->predictionsPlusFeaturesCount;
    int zone2Value = zone2->predictionsPlusFeaturesCount;

    if (zone1Value < zone2Value)
    {
        return -1;
    }

    if (zone1Value == zone2Value)
    {
        return 0;
    }

    return 1;
}

// ------------------------------------------------------------------------------------------------------------

int getPointZone(const double imagePoint[2], int zoneWidth, int zoneHeight, int width, int height)
{
    int zoneX = (int)imagePoint[0] / zoneWidth;
    int zoneY = (int)imagePoint[1] / zoneHeight;
    return zoneY * (width / zoneWidth) + zoneX;
}

// ------------------------------------------------------------------------------------------------------------

void buildImageMask( const VectorImageFeaturePrediction &inlierPredictions,
                     cv::Mat &imageMask )
{
    // Filtramos las zonas alrededor de cada prediccion
    size_t inlierPredictionsSize = inlierPredictions.size();

    cv::Scalar colorBlack(0, 0, 0);

    // Para cada prediccion, se dibuja dentro de la mascara su elipse de incertidumbre
    // De esta forma se previene el hecho de agregar nuevos features dentro de
    // elipses de busqueda de features anteriores
    for (int i = 0; i < inlierPredictionsSize; ++i)
    {
        ImageFeaturePrediction *inlierPrediction = inlierPredictions[i];

        drawUncertaintyEllipse2D( imageMask,
                                  cv::Point2d(inlierPrediction->imagePos[0], inlierPrediction->imagePos[1]),
                                  inlierPrediction->covarianceMatrix,
                                  2 * (imageMask.rows + imageMask.cols),
                                  colorBlack,
                                  true );
    }
}

// ------------------------------------------------------------------------------------------------------------
// Agrupa un conjunto de features y predicciones por zona, y devuelve el zoneInfo para cada zona.
void groupImageFeaturesAndPredictionsByZone( const VectorImageFeaturePrediction &featuresPrediction,
                                             std::vector<cv::KeyPoint> &imageKeypoints,
                                             cv::Mat &keypointsDescriptors,
                                             int zoneWidth, int zoneHeight,
                                             int totalWidth, int totalHeight,
                                             int zonesCount, ZoneInfo **zonesInfo )
{
    size_t imageKeypointsSize = imageKeypoints.size();
    for (uint i = 0; i < imageKeypointsSize; ++i)
    {
        cv::KeyPoint &currKeypoint = imageKeypoints[i];

        double imagePos[2] = {currKeypoint.pt.x, currKeypoint.pt.y};
        ImageFeatureMeasurement *imageFeature = new ImageFeatureMeasurement(imagePos, keypointsDescriptors.row(i));

        int currFeatureZoneId = getPointZone(imageFeature->imagePos, zoneWidth, zoneHeight, totalWidth, totalHeight);

        ZoneInfo *currZoneInfo = zonesInfo[currFeatureZoneId];
        currZoneInfo->candidateImageFeatureMeasurement.push_back(imageFeature);
        currZoneInfo->candidateImageFeatureMeasurementLeft++;
    }

    size_t featuresPredictionSize = featuresPrediction.size();
    for (uint i = 0; i < featuresPredictionSize; ++i)
    {
        int currPredZoneId = getPointZone(featuresPrediction[i]->imagePos, zoneWidth, zoneHeight, totalWidth, totalHeight);

        ZoneInfo *currZoneInfo = zonesInfo[currPredZoneId];
        currZoneInfo->featuresPrediction.push_back(featuresPrediction[i]);
        currZoneInfo->predictionsPlusFeaturesCount++;
    }
}

// ------------------------------------------------------------------------------------------------------------

// Esta funcion busca features considerando cada zona de la imagen e intenta distribuirlos de modo que cada una
// tenga a lo sumo un feature mas que las demas zonas.
// La cantidad de features en cada zona es limitante para esta heuristica, pero no es un problema.
// Es decir, por ejemplo, si una zona no tiene features, simplemente no la tiene en cuenta e intenta
// balancear la cantidad de features de las que si tengan.

void searchFeaturesByZone( const VectorImageFeaturePrediction &featuresPrediction,
                           std::vector<cv::KeyPoint> &imageKeypoints, cv::Mat &descriptors,
                           int zonesInARow, int zoneWidth, int zoneHeight,
                           cv::Mat& imageMask,
                           int imageFeaturesMaxSize,
                           VectorImageFeatureMeasurement &newImageFeatures )
{
    int zonesCount = zonesInARow * zonesInARow;

    // Inicializamos el ZoneInfo para cada zona.
    ZoneInfo **zonesInfo = new ZoneInfo *[zonesCount];

    for (int i = 0; i < zonesCount; ++i)
    {
        ZoneInfo *currZoneInfo = new ZoneInfo;
        currZoneInfo->candidateImageFeatureMeasurementLeft = 0;
        currZoneInfo->predictionsPlusFeaturesCount = 0;
        currZoneInfo->zoneId = i;

        zonesInfo[i] = currZoneInfo;
    }

    groupImageFeaturesAndPredictionsByZone( featuresPrediction, imageKeypoints, descriptors,
                                            zoneWidth, zoneHeight, imageMask.cols, imageMask.rows, zonesCount,
                                            zonesInfo );

    // Ordenamos el arreglo de ZoneInfo por su cantidad de predicciones tal que su media cae dentro de la zona
    qsort(zonesInfo, zonesCount, sizeof(ZoneInfo *), &sortCompare);

    // Convertimos el arreglo de ZoneInfo a lista para lograr mayor performance en la eliminacion
    std::list< ZoneInfo * > zoneInfoList;

    for (int i = 0; i < zonesCount; ++i)
    {
        zoneInfoList.push_back( zonesInfo[i] );
    }

    // Agregamos los features al resultado
    newImageFeatures.reserve(imageFeaturesMaxSize);

    double ellipseSize = ConfigurationManager::getInstance().ekfParams->detectNewFeaturesImageMaskEllipseSize;
    Matd newFeatMeasToAddEllipse( (Matd(2, 2) << ellipseSize, 0.0L,
                                                        0.0L, ellipseSize) );

    int zonesLeft = zonesCount;
    while(zonesLeft > 0 && imageFeaturesMaxSize > 0)
    {
        ZoneInfo *currZoneInfo = zoneInfoList.front();
        int &currZoneFeaturesLeft = currZoneInfo->candidateImageFeatureMeasurementLeft;

        // Si no hay mas features en la zona, la quitamos de la lista
        if (currZoneFeaturesLeft == 0)
        {
            zoneInfoList.pop_front();

#ifdef DEBUG_SHOW_NEW_FEATURES
            std::cout << "Se elimino la zona " << currZoneInfo->zoneId << " por no tener más features candidatos restantes." << std::endl;
            std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl << std::endl;
#endif
            zonesLeft--;
        }
        else
        {
            // Tomamos un feature de forma aleatoria
            int randKeypointIdx = rand() % currZoneFeaturesLeft;
            ImageFeatureMeasurement *imFeatMeasToAdd = currZoneInfo->candidateImageFeatureMeasurement[randKeypointIdx];

            int imFeatMeasToAddX = static_cast<int>(imFeatMeasToAdd->imagePos[0]);
            int imFeatMeasToAddY = static_cast<int>(imFeatMeasToAdd->imagePos[1]);
            if ( imageMask.at<uchar>(imFeatMeasToAddY, imFeatMeasToAddX) )
            {
                // Agregamos el feature al resultado y utilizamos el constructor por copia
                // Aca si que copiamos el descriptor, para poder devolverlo
                newImageFeatures.push_back( new ImageFeatureMeasurement(*imFeatMeasToAdd) );
                currZoneInfo->predictionsPlusFeaturesCount++;

#ifdef DEBUG_SHOW_NEW_FEATURES
                std::cout << "Agregamos feature en la zona " << currZoneInfo->zoneId << std::endl;
                std::cout << "Hay ahora " << currZoneInfo->predictionsPlusFeaturesCount << " predicciones y features" << std::endl;
                std::cout << "Quedan " << currZoneInfo->candidateImageFeatureMeasurementLeft << " feature candidatos por agregar" << std::endl;
                std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl << std::endl;
#endif

                // Reordeno la lista
                std::list< ZoneInfo * >::iterator itCurr = zoneInfoList.begin();
                std::list< ZoneInfo * >::iterator itNext = zoneInfoList.begin();
                std::list< ZoneInfo * >::iterator itEnd = zoneInfoList.end();
                itNext++;

                // Seguimos recorriendo
                while(itNext != itEnd)
                {
                    ZoneInfo *nextZoneInfo = static_cast<ZoneInfo *>(*itNext);

                    // Si la cantidad de features y predicciones de la zona actual
                    // supera o iguala a la cantidad de features y predicciones de la siguiente zona en la lista,
                    // cambio el orden de la zona actual por la siguiente, para asegurar el orden total
                    // de las zonas con respecto a la cantidad de features
                    if (currZoneInfo->predictionsPlusFeaturesCount >= nextZoneInfo->predictionsPlusFeaturesCount)
                    {
                        (*itNext) = currZoneInfo;
                        (*itCurr) = nextZoneInfo;
                    }
                    else
                    {
                        break;
                    }

                    itCurr++;
                    itNext++;
                }

                // Actualizamos la mascara
                drawUncertaintyEllipse2D( imageMask,
                                          cv::Point2d(imFeatMeasToAdd->imagePos[0], imFeatMeasToAdd->imagePos[1]),
                                          newFeatMeasToAddEllipse,
                                          2 * (imageMask.cols + imageMask.rows),
                                          cv::Scalar(0, 0, 0),
                                          true );

                // Actualizamos la cantidad de features buscada
                imageFeaturesMaxSize--;
            }

            // Nos aseguramos que el mismo feature no sea reelecto
            currZoneFeaturesLeft--;
            currZoneInfo->candidateImageFeatureMeasurement[randKeypointIdx] =
                currZoneInfo->candidateImageFeatureMeasurement[currZoneFeaturesLeft];
            
            delete imFeatMeasToAdd;
        }
    }

    // Liberamos toda la memoria auxiliar utilizada
    for (int i = 0; i < zonesCount; ++i)
    {
        ZoneInfo *currZoneInfo = zonesInfo[i];

        for (int j = 0; j < currZoneInfo->candidateImageFeatureMeasurementLeft; ++j)
        {
            delete currZoneInfo->candidateImageFeatureMeasurement[j];
        }
        
        delete currZoneInfo;
    }

    delete [] zonesInfo;
}

// ------------------------------------------------------------------------------------------------------------

void detectNewImageFeatures( const cv::Mat image,
                             const VectorImageFeaturePrediction &featuresPrediction,
                             uint newImageFeaturesMaxSize,
                             VectorImageFeatureMeasurement &newImageFeatures )
{
    newImageFeatures.clear();

    // Calculamos el tamaño de las zonas de la imágen según los parámetros configurados
    ConfigurationManager& configManager = ConfigurationManager::getInstance();
    int zonesInARow = exp2f(configManager.ekfParams->detectNewFeaturesImageAreasDivideTimes);

    int zoneWidth = image.cols / zonesInARow;
    int zoneHeight = image.rows / zonesInARow;

    // Construimos la mascara para buscar features solamente en zonas poco densas
    cv::Mat imageMask( cv::Mat::ones(image.rows, image.cols, CV_8UC1) * 255 );
    buildImageMask( featuresPrediction, imageMask );

    // Detectamos features
    std::vector<cv::KeyPoint> imageKeypoints;
    configManager.featureDetector->detect(image, imageKeypoints, imageMask);

    // Extraemos descriptores
    cv::Mat descriptors;
    configManager.descriptorExtractor->compute(image, imageKeypoints, descriptors);

    // Caso particular: la cantidad de features encontrados no supera los pedidos
    size_t imageKeypointsSize = imageKeypoints.size();

#ifdef DEBUG
    std::cout << "Cantidad de features detectados al agregar nuevos: " << imageKeypointsSize << std::endl;
#endif

    if (imageKeypointsSize <= newImageFeaturesMaxSize)
    {
        double imagePos[2];
        for (int i = 0; i < imageKeypointsSize; ++i)
        {
            cv::KeyPoint &currKeypoint = imageKeypoints[i];

            imagePos[0] = currKeypoint.pt.x;
            imagePos[1] = currKeypoint.pt.y;

            newImageFeatures.push_back( new ImageFeatureMeasurement( imagePos,
                                                                     descriptors.row(i) ) );
        }
    }
    else
    {
        // Buscamos nuevos features intentando que esten
        // lo mejor distribuidos posible en la imagen
        searchFeaturesByZone( featuresPrediction, imageKeypoints, descriptors,
                              zonesInARow, zoneWidth, zoneHeight,
                              imageMask,
                              newImageFeaturesMaxSize, newImageFeatures );
    }

#ifdef DEBUG_SHOW_NEW_FEATURES
    cv::Mat imageCopy;
    image.copyTo(imageCopy);
    for (int i = 1; i < zonesInARow; ++i)
    {
        cv::line(imageCopy, cv::Point(i * zoneWidth, 0), cv::Point(i * zoneWidth, imageCopy.rows), cv::Scalar(0, 255, 0));
        cv::line(imageCopy, cv::Point(0, i * zoneHeight), cv::Point(imageCopy.cols, i * zoneHeight), cv::Scalar(0, 255, 0));
    }

    int featuresPredictionSize = featuresPrediction.size();
    for (int i = 0; i < featuresPredictionSize; ++i)
    {
        ImageFeaturePrediction *currFeaturePrediction = featuresPrediction[i];

        drawUncertaintyEllipse2D( imageCopy,
                                  cv::Point2f(currFeaturePrediction->imagePos[0], currFeaturePrediction->imagePos[1]),
                                  currFeaturePrediction->covarianceMatrix,
                                  2 * (image.cols + image.rows),
                                  cv::Scalar(0, 255, 0),
                                  false );
    }

    cv::namedWindow("Busqueda de nuevos features: mascara");
    cv::imshow("Busqueda de nuevos features: mascara", imageMask);
    cv::waitKey(0);

    for (int i = 0; i < newImageFeatures.size(); ++i)
    {
        drawPoint(imageCopy, newImageFeatures[i]->imagePos, cv::Scalar(0, 255, 255));
    }

    cv::namedWindow("Busqueda de nuevos features: imagen con nuevos features");
    cv::imshow("Busqueda de nuevos features: imagen con nuevos features", imageCopy);
    cv::waitKey(0);

    // Se borran todas las ventanas creadas
    cv::destroyWindow("Busqueda de nuevos features: mascara");
    cv::destroyWindow("Busqueda de nuevos features: imagen con nuevos features");
#endif
}

// ------------------------------------------------------------------------------------------------------------
