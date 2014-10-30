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

#ifndef __MODULES_GUI_DRAW_H__
#define __MODULES_GUI_DRAW_H__

#include <opencv2/highgui/highgui.hpp>

#include "../Core/Base.h"
#include "../1PointRansacEKF/State.h"
#include "../1PointRansacEKF/ImageFeaturePrediction.h"

#ifdef DEBUG_SHOW_3D_MAP
#include <pcl/visualization/pcl_visualizer.h>
#endif

#if defined(DEBUG_SHOW_IMAGES)
#include "../1PointRansacEKF/Matching.h"
#endif

// Se define el tamaño de la ventana que se usa para mostrar la info planar
#define PLANAR_WINDOW_SIZE_X     800
#define PLANAR_WINDOW_SIZE_Y     480



enum PlanarInformationPerspective
{
    PLANAR_INFORMATION_XY,
    PLANAR_INFORMATION_XZ,
    PLANAR_INFORMATION_YZ,
};

// Dibuja una elipse en una imagen
// Parametros:
// img              = imagen donde se quiere dibujar la elipse
// center           = centro de la elipse (en pixels) con respecto al origen de coordenadas en la parte superior izquierda de la imagen
// covarianceMatrix = matriz de covarianza que representa las dimensiones de la elipse
// maxAxesSize      = tamaño maximo de los ejes de la elipse
// ellipseColor     = color con el que se quiere pintar el borde (y posiblemente relleno) de la elipse
// fillEllipse      = se quiere rellenar la elipse con el color ellipseColor?
void drawUncertaintyEllipse2D( cv::Mat &img,
                               cv::Point2f center,
                               const Matd &covarianceMatrix,
                               int maxAxesSize,
                               cv::Scalar ellipseColor,
                               bool fillEllipse );

void drawPoint( cv::Mat &img, double *point, cv::Scalar color );

void drawPlanarInformation(const State &state, const Matd covariance, const double *cameraPreviousPosition,
                           PlanarInformationPerspective perspective, cv::Mat &cameraPositionImage, cv::Mat &featuresPositionImage, cv::Mat &cameraPositionEllipse);

void drawPrediction( const cv::Mat &image,
                     const VectorImageFeaturePrediction &predictedDistortedFeatures,
                     const VectorMapFeature &stateMapFeatures,
                     cv::Mat &result );

#ifdef DEBUG_SHOW_3D_MAP
boost::shared_ptr<pcl::visualization::PCLVisualizer> create3DMapVisualizer(const State &state);
void draw3DMap(const State &state, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
#endif

#if defined(DEBUG_SHOW_IMAGES)
void showMatches( const cv::Mat &image,
                  const VectorMapFeature &mapFeatures,
                  const VectorFeatureMatch &matches,
                  std::string windowName );
#endif

#ifdef DEBUG_SHOW_RANSAC_INFO
void showRansacInliers(const VectorFeatureMatch &matches,
                       const VectorImageFeaturePrediction &predictedImageFeatures,
                       const std::vector<int> &supportIndexesInMatchesVector,
                       const FeatureMatch *match,
                       std::string windowName);

void showRescueOutliers(const VectorFeatureMatch &rescuedMatches,
                        const VectorImageFeaturePrediction &rescuedPredictions,
                        std::string windowName);
#endif

#endif // __MODULES_GUI_DRAW_H__
