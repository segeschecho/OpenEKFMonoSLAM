//Copyright (C) 2013 Sergio E. Gonzalez and Emiliano D. González
//Facultad de Ciencias Exactas y Naturales, Universidad de Buenos Aires, Buenos Aires, Argentina.
 
//C/C++, Java and XML/YML code for EKF SLAM from a monocular sequence.

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
//along with OpenEKFMonoSLAM.  If not, see <http://www.gnu.org/licenses/>.

//If you use this code for academic work, please reference:
//GONZALEZ, S.; E. GONZÁLEZ; M. NITSCHE; P. DE CRISTÓFORIS. "Odometría Visual para Robots Móviles Utilizando Smartphones como Unidad de Sensado y Procesamiento". En: Jornadas Argentinas de Robótica, 8as : 2014 : Ciudad Autónoma de Buenos Aires. Actas : VIII Jornadas Argentinas de Robótica . (8 : Ciudad Autónoma de Buenos Aires, 12-14 de noviembre 2014).

//Authors:    Sergio E. Gonzalez - segonzalez@dc.uba.ar
//            Emiliano D. González - edgonzalez@dc.uba.ar

//Departamento de Computación
//Facultad de Ciencias Exactas y Naturales
//Universidad de Buenos, Buenos Aires, Argentina
//Date: June 2013

#include "../Core/EKFMath.h"
#include "Draw.h"
#include "../1PointRansacEKF/State.h"
#include "../1PointRansacEKF/CommonFunctions.h"
#include "../Configuration/ConfigurationManager.h"

#ifdef DEBUG_SHOW_3D_MAP
#include "../1PointRansacEKF/State.h"
#endif

void drawUncertaintyEllipse2D( cv::Mat &img,
                               cv::Point2f center,
                               const Matd& covarianceMatrix,
                               int maxAxesSize,
                               cv::Scalar ellipseColor,
                               bool fillEllipse )
{
    double rotationAngle;
    cv::Size2f axesSize;
    cv::Point intCenter(center.x, center.y);

    matrix2x2ToUncertaintyEllipse2D(covarianceMatrix, axesSize, rotationAngle);

    cv::Size axesSizeInt(MIN(axesSize.width, maxAxesSize), MIN(axesSize.height, maxAxesSize));
    if (fillEllipse)
    {
        cv::ellipse(img, intCenter, axesSizeInt, RAD_TO_DEG(rotationAngle), 0, 360, ellipseColor, -1);
    }
    else
    {
        cv::ellipse(img, intCenter, axesSizeInt, RAD_TO_DEG(rotationAngle), 0, 360, ellipseColor);
    }
}

void drawPoint( cv::Mat &img, double *point, cv::Scalar color )
{
    cv::line(img, cv::Point((int)point[0], (int)point[1] - 2), cv::Point((int)point[0], (int)point[1] + 2), color);
    cv::line(img, cv::Point((int)point[0] + 2, (int)point[1]), cv::Point((int)point[0] - 2, (int)point[1]), color);
}

void adjustScale(const double *originalXYZPosition, PlanarInformationPerspective perspective, double *adjustedPosition)
{
    CameraCalibration *camCalib = ConfigurationManager::getInstance().cameraCalibration;

    if (perspective == PLANAR_INFORMATION_XY)
    {
        adjustedPosition[0] = originalXYZPosition[0]/camCalib->dx + PLANAR_WINDOW_SIZE_X/2;
        adjustedPosition[1] = originalXYZPosition[1]/camCalib->dy + PLANAR_WINDOW_SIZE_Y*0.7;

//        adjustedPosition[0] = originalXYZPosition[0]*500 + PLANAR_WINDOW_SIZE_X/2;
//        adjustedPosition[1] = originalXYZPosition[1]*500 + PLANAR_WINDOW_SIZE_Y*0.7;
    }
    else if (perspective == PLANAR_INFORMATION_XZ)
    {
        adjustedPosition[0] = originalXYZPosition[0]/camCalib->dx + PLANAR_WINDOW_SIZE_X/2;
        adjustedPosition[1] = originalXYZPosition[2]/camCalib->dy + PLANAR_WINDOW_SIZE_Y*0.9;
    }
    else
    {
        adjustedPosition[0] = originalXYZPosition[1]/camCalib->dx + 50;
        adjustedPosition[1] = originalXYZPosition[2]/camCalib->dy + 200;
    }
}

void drawPlanarInformation(const State &state, const Matd covariance, const double *cameraPreviousPosition,
                           PlanarInformationPerspective perspective, cv::Mat &cameraPositionImage, cv::Mat &featuresPositionImage, cv::Mat &cameraPositionEllipse)
{
    double actualPosition[2] = {0};
    adjustScale(state.position, PLANAR_INFORMATION_XY, actualPosition);

    double previousPosition[2] = {0};
    adjustScale(cameraPreviousPosition, PLANAR_INFORMATION_XY, previousPosition);

    cv::Scalar colorGreen(0, 255, 0);
    cv::Scalar colorRed(0, 0, 255);


    cv::line(cameraPositionImage, cv::Point((int)previousPosition[0], (int)previousPosition[1]), cv::Point((int)actualPosition[0], (int)actualPosition[1]), colorGreen);

    // Se dibujan los features.
    for (uint i = 0; i < state.mapFeatures.size(); ++i)
    {
        MapFeature *f = state.mapFeatures[i];
        double depthPoint[3] = {0};

        if (f->featureType == MAPFEATURE_TYPE_INVERSE_DEPTH)
        {
            changeInverseDepthToDepth(f->position, depthPoint);
        }

        double imagePosition[2] = {0};

        adjustScale(depthPoint, PLANAR_INFORMATION_XY, imagePosition);

        drawPoint(featuresPositionImage, imagePosition, colorRed);
    }

    // Se muestra la elipse de incertidumbre de la posicion de la camara magnificada.
    cv::Scalar colorLightBlue(255, 173, 85);
    Matd adjustedCovariance;
    Matd(covariance(cv::Range(0, 2), cv::Range(0, 2))).copyTo(adjustedCovariance);

    double scaleCoeficient = 1000000;
    adjustedCovariance[0][0] *= scaleCoeficient;
    adjustedCovariance[0][1] *= scaleCoeficient;
    adjustedCovariance[1][0] *= scaleCoeficient;
    adjustedCovariance[1][1] *= scaleCoeficient;

    drawUncertaintyEllipse2D( cameraPositionEllipse,
                 cv::Point2f(static_cast<float>(actualPosition[0]), static_cast<float>(actualPosition[1])),
                 adjustedCovariance,
                 9999999,
                 colorLightBlue,
                 false );

    std::cout << "Se muestran en la ventana: " << state.mapFeatures.size() << " features" << std::endl;
}

#ifdef DEBUG_SHOW_3D_MAP

static double cameraPointCloudLastPos[3];

pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPointCloud(const State &state)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr( new pcl::PointCloud<pcl::PointXYZRGB> );

    size_t mapFeaturesSize = state.mapFeatures.size();
    for (int i = 0; i < mapFeaturesSize; ++i)
    {
        double featurePosXYZ[3];
        MapFeature *currMapFeature = state.mapFeatures[i];

        if (currMapFeature->positionDimension == 3)
        {
            featurePosXYZ[0] = currMapFeature->position[0];
            featurePosXYZ[1] = currMapFeature->position[1];
            featurePosXYZ[2] = currMapFeature->position[2];
        } else {
            changeInverseDepthToDepth(currMapFeature->position, featurePosXYZ);
        }

        uint32_t red = 255 << 16;
        uint32_t green = 255 << 8;

        pcl::PointXYZRGB point;
        point.x = static_cast<float>(featurePosXYZ[0]);
        point.y = static_cast<float>(featurePosXYZ[1]);
        point.z = static_cast<float>(featurePosXYZ[2]);
        point.rgb = *reinterpret_cast<float*>((currMapFeature->featureType == MAPFEATURE_TYPE_DEPTH) ? &green : &red);

        pointCloudPtr->points.push_back (point);
    }

    pointCloudPtr->width = (int)pointCloudPtr->points.size();
    pointCloudPtr->height = 1;

    return pointCloudPtr;
}

void addCurrentCameraPos(const State &state, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    Eigen::Affine3f transformation(Eigen::Translation<float,3>(state.position[0], state.position[1], state.position[2]) *
                                   Eigen::Quaternionf(state.orientation[0], state.orientation[1], state.orientation[2], state.orientation[3]));

    viewer->removeCoordinateSystem();
    viewer->addCoordinateSystem(0.5f, transformation);

    pcl::PointXYZ p1, p2;

    p1.x = static_cast<float>(cameraPointCloudLastPos[0]);
    p1.y = static_cast<float>(cameraPointCloudLastPos[1]);
    p1.z = static_cast<float>(cameraPointCloudLastPos[2]);

    p2.x = static_cast<float>(state.position[0]);
    p2.y = static_cast<float>(state.position[1]);
    p2.z = static_cast<float>(state.position[2]);

    std::ostringstream s;
    s << "path" << "(" << p1.x << ", " << p1.y << ", " << p1.z << ")";

    viewer->addLine(p1, p2, 1.0f, 0, 0, s.str());

    for(int i = 0; i < 3; ++i)
    {
        cameraPointCloudLastPos[i] = state.position[i];
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> create3DMapVisualizer(const State &state)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer("3D Viewer") );

    // Add camera position and orientation to the map
    addCurrentCameraPos(state, viewer);

    // Add point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr = createPointCloud(state);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudPtr);
    viewer->addPointCloud<pcl::PointXYZRGB>(pointCloudPtr, rgb, "map");

    // Init other parameters
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map");
//    viewer->setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0f, 0, "map");
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.5f);

    // Add initial camera position
    for(int i = 0; i < 3; ++i)
    {
        cameraPointCloudLastPos[i] = state.position[i];
    }

    Eigen::Affine3f transformation(Eigen::Translation<float,3>(state.position[0], state.position[1], state.position[2]) *
                                   Eigen::Quaternionf(state.orientation[0], state.orientation[1], state.orientation[2], state.orientation[3]));

    viewer->addCoordinateSystem(0.5f, transformation);
    viewer->setCameraPosition(0, 0, -3, 0, -1, 0);

    return viewer;
}

void draw3DMap(const State &state, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr = createPointCloud(state);
    viewer->updatePointCloud(pointCloudPtr, "map");

    addCurrentCameraPos(state, viewer);
}
#endif


// ------------------------------------------------------------------------------------------------------------

void drawPrediction( const cv::Mat &image,
                     const VectorImageFeaturePrediction &predictedDistortedFeatures,
                     const VectorMapFeature &stateMapFeatures,
                     cv::Mat &result )
{
    result = cv::Mat(image.rows, image.cols, image.type());
    image.copyTo(result);

    size_t predictedFeaturesCount = predictedDistortedFeatures.size();
    cv::Scalar colorGreen(0, 255, 0);
    cv::Scalar colorRed(0, 0, 255);
    cv::Scalar colorText(0, 255, 255);
    cv::Scalar colorShadow(150, 150, 150);

    for (int i = 0; i < predictedFeaturesCount; ++i)
    {
        Matd &covarianceMatrix = predictedDistortedFeatures[i]->covarianceMatrix;
        double *predictedFeaturePos = predictedDistortedFeatures[i]->imagePos;

        int featureIndex = predictedDistortedFeatures[i]->featureIndex;
        bool isInverseDepth = stateMapFeatures[featureIndex]->featureType == MAPFEATURE_TYPE_INVERSE_DEPTH;

        // Dibujo la posicion del feature predicho y matcheado
        drawPoint(result, predictedFeaturePos, isInverseDepth ? colorRed : colorGreen);

        // Dibujo la elipse del feature predicho y matcheado
        drawUncertaintyEllipse2D( result,
                                  cv::Point2f(static_cast<float>(predictedFeaturePos[0]), static_cast<float>(predictedFeaturePos[1])),
                                  covarianceMatrix,
                                  9999999,
                                  isInverseDepth ? colorRed : colorGreen,
                                  false );
        
        std::ostringstream s;
        s << featureIndex;

        cv::Point2d shadowOrg(predictedFeaturePos[0] + 1, predictedFeaturePos[1] + 1);
        putText( result, s.str(), shadowOrg, 1, 1.3f, colorShadow);

        cv::Point2d txtOrg(predictedFeaturePos[0], predictedFeaturePos[1]);
        putText( result, s.str(), txtOrg, 1, 1.3f, colorText);
    }
    
    std::cout << std::endl;
}

// ------------------------------------------------------------------------------------------------------------

#if defined(DEBUG_SHOW_IMAGES)
void showMatches( const cv::Mat &image,
                  const VectorMapFeature &mapFeatures,
                  const VectorFeatureMatch &matches,
                  std::string windowName )
{
    cv::Mat imageCopy(image.rows, image.cols, image.type());
    image.copyTo(imageCopy);

    size_t matchesSize = matches.size();
    cv::Scalar ellipseColor(0, 255, 0);

    for (int i = 0; i < matchesSize; ++i)
    {
        FeatureMatch *match = matches[i];

        double *detectedFeaturePos = match->imagePos;
        double *mapFeaturePos = mapFeatures[match->featureIndex]->lastImagePos;

//        std::cout << "Feature detectado matcheado: " << detectedFeaturePos[0] << " "
//                                                     << detectedFeaturePos[1] << std::endl;
//
//        std::cout << "Con feature que previamente se vio en la posición: " << mapFeaturePos[0] << " "
//                                                                           << mapFeaturePos[1] << std::endl;
//
//        std::cout << std::endl << std::endl << std::endl;

        cv::Scalar detectedFeatureColor;
        if (mapFeaturePos[0] < 0 || mapFeaturePos[1] < 0)
        {
            detectedFeatureColor = cv::Scalar(255,0,0);
        }
        else
        {
            detectedFeatureColor = cv::Scalar(0,255,0);

            // Dibujo la posicion del feature del mapa (ultima posicion)
            cv::line(imageCopy, cv::Point((int)mapFeaturePos[0], (int)mapFeaturePos[1] - 2),
                     cv::Point((int)mapFeaturePos[0], (int)mapFeaturePos[1] + 2), cv::Scalar(255,0,255));
            cv::line(imageCopy, cv::Point((int)mapFeaturePos[0] + 2, (int)mapFeaturePos[1]),
                     cv::Point((int)mapFeaturePos[0] - 2, (int)mapFeaturePos[1]), cv::Scalar(255,0,255));

            // Dibujo la linea desde el feature detectado al feature predicho
            cv::line(imageCopy, cv::Point((int)detectedFeaturePos[0], (int)detectedFeaturePos[1]),
                     cv::Point((int)mapFeaturePos[0], (int)mapFeaturePos[1]), cv::Scalar(0,255,0));
        }

        // Dibujo la posicion del feature detectado y matcheado
        cv::line(imageCopy, cv::Point((int)detectedFeaturePos[0], (int)detectedFeaturePos[1] - 2),
                 cv::Point((int)detectedFeaturePos[0], (int)detectedFeaturePos[1] + 2), detectedFeatureColor);
        cv::line(imageCopy, cv::Point((int)detectedFeaturePos[0] + 2, (int)detectedFeaturePos[1]),
                 cv::Point((int)detectedFeaturePos[0] - 2, (int)detectedFeaturePos[1]), detectedFeatureColor);
    }

    // Mostramos la imagen
    cv::namedWindow(windowName);
    cv::imshow(windowName, imageCopy);
    cv::waitKey(1);
}

#endif

#ifdef DEBUG_SHOW_RANSAC_INFO
//--------------------------------------------------------------------------------------------------------
// El objetivo de esta funcion es mostrar los puntos inliers en cada una de las iteraciones de RANSAC
// El orden de los matches y las predicciones es el mismo
//--------------------------------------------------------------------------------------------------------
void showRansacInliers(const VectorFeatureMatch &matches,
                       const VectorImageFeaturePrediction &predictedImageFeatures,
                       const std::vector<int> &supportIndexesInMatchesVector,
                       const FeatureMatch *match,
                       std::string windowName)
{
    CameraCalibration *camCalib = ConfigurationManager::getInstance().cameraCalibration;

    cv::Size size(camCalib->pixelsX, camCalib->pixelsY);
    cv::Mat image = cv::Mat::zeros(size, CV_8UC3);

    cv::Scalar colorOrange(0, 120, 255);
    cv::Scalar colorRed(0, 0, 255);
    cv::Scalar colorLightBlue(255, 174, 0);
    cv::Scalar colorViolet(252, 0, 255);
    cv::Scalar colorBlue(255, 90, 52);

    // se arma una mascara para tener los inliers y los outliers para mostrar
    std::vector<bool> mask(matches.size());
    int j = 0;
    for (uint i = 0; i < matches.size() && j < supportIndexesInMatchesVector.size(); ++i)
    {
        if (i == supportIndexesInMatchesVector[j])
        {
            mask[i] = true;
            j++;
        }
        else
        {
            mask[i] = false;
        }
    }

    for (uint i = 0; i < mask.size(); ++i)
    {
        FeatureMatch *match = matches[i];

        // Se busca la prediccion asociada al match
        bool wasFound = false;
        int pIndex = 0;
        while (!wasFound && pIndex < predictedImageFeatures.size())
        {
            ImageFeaturePrediction *prediction = predictedImageFeatures[pIndex];

            if(prediction->featureIndex == match->featureIndex)
            {
                // Si se encontro la prediccion para el feature entonces se pinta en la imagen.

                if (mask[i])
                {
                    // Si el punto es inlier entonces se pintan de otro color
                    drawPoint( image, prediction->imagePos, colorViolet );
                    drawPoint( image, match->imagePos, colorRed );
                }
                else
                {
                    drawPoint( image, prediction->imagePos,  colorLightBlue);
                    drawPoint( image, match->imagePos, colorBlue );
                }

                // Se dibuja la distancia entre la prediccion y el match dentro de la imagen.
                double xDist = match->imagePos[0] - prediction->imagePos[0];
                double yDist = match->imagePos[1] - prediction->imagePos[1];
                double dist = sqrt(xDist*xDist + yDist*yDist);

                cv::Point textOrgDist(match->imagePos[0], match->imagePos[1] - 5);
                std::ostringstream s;
                s << std::setprecision(4) << dist;
                putText( image, s.str(), textOrgDist, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3f, cv::Scalar::all(255));

                wasFound = true;
            }

            pIndex++;
        }
    }

    cv::Point center(match->imagePos[0], match->imagePos[1]);
    circle(image, center, 7, colorViolet);

    // Se muestra la imagen
    cv::namedWindow(windowName);
    cv::imshow(windowName, image);
    cv::waitKey(0);
}

//--------------------------------------------------------------------------------------------------------
// Se muestran los matches y predicciones salvadas luego de hacer ransac y update
//--------------------------------------------------------------------------------------------------------
void showRescueOutliers(const VectorFeatureMatch &rescuedMatches,
                        const VectorImageFeaturePrediction &rescuedPredictions,
                        std::string windowName)
{
    CameraCalibration *camCalib = ConfigurationManager::getInstance().cameraCalibration;

    cv::Size size(camCalib->pixelsX, camCalib->pixelsY);
    cv::Mat image = cv::Mat::zeros(size, CV_8UC3);

    cv::Scalar colorLightBlue(255, 174, 0);
    cv::Scalar colorBlue(255, 90, 52);

    for (uint i = 0; i < rescuedMatches.size(); ++i)
    {
        FeatureMatch *match = rescuedMatches[i];
        ImageFeaturePrediction *prediction = rescuedPredictions[i];

        drawPoint( image, match->imagePos, colorBlue );
        drawPoint( image, prediction->imagePos,  colorLightBlue);
    }

    cv::namedWindow(windowName);
    cv::imshow(windowName, image);
}

#endif

