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

#ifndef __MODULES_CONFIGURATION_CAMERACALIBRATIONCONFIGURATION_CAMERACALIBRATION_H__
#define __MODULES_CONFIGURATION_CAMERACALIBRATIONCONFIGURATION_CAMERACALIBRATION_H__

#include "../../../Core/Base.h"

class CameraCalibration
{
    public:
        // Constructor and Destructor
        CameraCalibration(){};
        ~CameraCalibration(){};

        std::string cameraName;
    
        int pixelsX;
        int pixelsY;

        double fx;
        double fy;
        double k1;
        double k2;
        double cx;
        double cy;
        double dx;
        double dy;

        double pixelErrorX;
        double pixelErrorY;
        
        double angularVisionX;
        double angularVisionY;
};

// Show function
//std::ostream& operator<<( std::ostream& os, const CameraCalibration& cameraCalibration)
//{
//    os << std::setprecision(20)
//       << cameraCalibration.cameraName << std::endl
//       << "Pixels X = " << cameraCalibration.pixelsX << std::endl
//       << "Pixels Y = " << cameraCalibration.pixelsY << std::endl
//       << "FX = " << cameraCalibration.fx << std::endl
//       << "FY = " << cameraCalibration.fy << std::endl
//       << "K1 = " << cameraCalibration.k1 << std::endl
//       << "K2 = " << cameraCalibration.k2 << std::endl
//       << "CX = " << cameraCalibration.cx << std::endl
//       << "CY = " << cameraCalibration.cy << std::endl
//       << "DX = " << cameraCalibration.dx << std::endl
//       << "DY = " << cameraCalibration.dy << std::endl
//       << "Pixel error X = " << cameraCalibration.pixelErrorX << std::endl
//       << "Pixel error Y = " << cameraCalibration.pixelErrorY << std::endl
//       << "Angular Vision X = " << cameraCalibration.angularVisionX << std::endl
//       << "Angular Vision Y = " << cameraCalibration.angularVisionY << std::endl;
//
//    return os;
//}

#endif // __MODULES_CONFIGURATION_CAMERACALIBRATIONCONFIGURATION_CAMERACALIBRATION_H__
