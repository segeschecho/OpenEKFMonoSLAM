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

#include "../../modules/Core/Base.h"
#include "../../modules/Configuration/ConfigurationManager.h"
#include "../../modules/ImageGenerator/CameraImageGenerator.h"
#include "../../modules/ImageGenerator/FileSequenceImageGenerator.h"
#include "../../modules/ImageGenerator/VideoFileImageGenerator.h"
#include "../../modules/1PointRansacEKF/State.h"
#include "../../modules/1PointRansacEKF/EKF.h"
#include "../../modules/Gui/Draw.h"
#include "../../modules/1PointRansacEKF/HandMatching.h"
#include "../../modules/Core/Timer.h"

#include "../../modules/1PointRansacEKF/MapManagement.h"

int main(int argc, const char * argv[])
{
    Timer timer;
    timer.start();

    FileSequenceImageGenerator generator(argv[2], "", "png", 90, 6550);

    //VideoFileImageGenerator generator(argv[2]);

    cv::Mat image;

    generator.init();

    // path al archivo de configuracion, path de salida
    std::string outputPath("");
    if (argc > 3)
    {
        outputPath = argv[3];
    }
    else
    {
        outputPath = "";
    }

    EKF extendedKalmanFilter(argv[1], outputPath.c_str());

    // para la primer imagen, se inicializa Kalman Filter
    // Se hace un sensado y se agregan todos los features que se detectan en la imagen.
    image = generator.getNextImage();

#ifdef DEBUG
    int stepCount = 0;
    std::cout << "~~~~~~~~~~~~ STEP 0 ~~~~~~~~~~~~" << std::endl;
#endif

    if (!image.empty())
    {
        extendedKalmanFilter.init(image);
    }
    else
    {
        std::cout << "No se puede iniciar Kalman Filter dado que no hay imagenes disponibles." << std::endl;
        return 0;
    }

#ifdef DEBUG
    std::cout << extendedKalmanFilter.state << std::endl;
#endif

#ifdef DEBUG_SHOW_3D_MAP
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = create3DMapVisualizer(extendedKalmanFilter.state);
#endif

#ifdef DEBUG_SHOW_PLANAR_INFO
    double cameraPreviousPosition[3];

    cameraPreviousPosition[0] = state.position[0];
    cameraPreviousPosition[1] = state.position[1];
    cameraPreviousPosition[2] = state.position[2];
    // Se crean las ventanas y se muestran las posciones
    std::string windowName = "Informacion planar";
    cv::Size size(PLANAR_WINDOW_SIZE_X, PLANAR_WINDOW_SIZE_Y);

    cv::Mat cameraPositionImage(cv::Mat::zeros(size, CV_8UC3));
    cv::Mat cameraPositionEllipse(cv::Mat::zeros(size, CV_8UC3));
    cv::Mat featuresPositionImage(cv::Mat::zeros(size, CV_8UC3));
    cv::namedWindow(windowName);

    std::string windowNameActualImage = "Imagen actual";
    //cv::namedWindow(windowNameActualImage);

    drawPlanarInformation(state, covariance, cameraPreviousPosition, PLANAR_INFORMATION_XY, cameraPositionImage, featuresPositionImage, cameraPositionEllipse);

    cv::imshow(windowName, cv::Mat(cameraPositionImage + featuresPositionImage));
//    cv::imshow(windowNameActualImage, image);
    cv::waitKey(0);
#endif

    image = generator.getNextImage();

    while(!image.empty())
    {
#ifdef DEBUG
        std::cout << std::endl << std::endl;
        std::cout << "~~~~~~~~~~~~ STEP " << stepCount++ << " ~~~~~~~~~~~~" << std::endl;
#endif

        // para cada imagen recibida se hace un paso del EKF
        extendedKalmanFilter.step(image);

#ifdef DEBUG
        // mostramos el estado
        std::cout << extendedKalmanFilter.state << std::endl;
#endif

#ifdef DEBUG_SHOW_3D_MAP
        draw3DMap(extendedKalmanFilter.state, viewer);
        viewer->spinOnce(100);
#endif
#ifdef DEBUG_SHOW_PLANAR_INFO
        cv::Mat(cv::Mat::zeros(size, CV_8UC3)).copyTo(featuresPositionImage);
        cv::Mat(cv::Mat::zeros(size, CV_8UC3)).copyTo(cameraPositionEllipse);

        // Se muestra la posicion en 2 dimensiones.
        drawPlanarInformation(state, covariance, cameraPreviousPosition, PLANAR_INFORMATION_XY, cameraPositionImage, featuresPositionImage, cameraPositionEllipse);

        cv::imshow(windowName, cv::Mat(cameraPositionImage + featuresPositionImage + cameraPositionEllipse));
        cv::waitKey(1);
//        cv::imshow(windowNameActualImage, image);

        // Se actualiza la posicion anterior de la camara
        cameraPreviousPosition[0] = state.position[0];
        cameraPreviousPosition[1] = state.position[1];
        cameraPreviousPosition[2] = state.position[2];
#endif

#if defined(DEBUG_SHOW_IMAGES)
        // waitKey global para ir paso a paso o hacer todo de corrido
        cv::waitKey(1);
#endif

        image = generator.getNextImage();
    }

    std::cout << "Tiempo de ejecución total: "<< timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
    timer.stop();

#if defined(DEBUG_SHOW_IMAGES)
    cv::destroyAllWindows();
#endif

    return 0;
}
