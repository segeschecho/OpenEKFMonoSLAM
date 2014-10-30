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

#ifndef __MODULES_IMAGEGENERATOR_VIDEOFILEIMAGEGENERATOR_H__
#define __MODULES_IMAGEGENERATOR_VIDEOFILEIMAGEGENERATOR_H__

#include "ImageGenerator.h"
#include "../Core/Base.h"
#include "../Core/Timer.h"

// ---------------------------------------------------------------------------------
// Esta clase puede dar frames uno a uno, o también puede simular una fuente
// de video stream (como una camara con vista previa) y perder frames dependiendo
// de cuanto tiempo se tarde en llamar de nuevo a getNextImage
// ---------------------------------------------------------------------------------
class VideoFileImageGenerator : public ImageGenerator
{
    public:
        // Constructors and Destructor
        VideoFileImageGenerator();
        VideoFileImageGenerator(std::string pathToVideo, bool frameToFrame = false);
        virtual ~VideoFileImageGenerator();

        // Inherited Methods
        virtual void init();
        virtual cv::Mat& getNextImage();

    private:
        // Attributes
        std::string _path;
        uint _totalFrames;
        cv::VideoCapture _captureReference;
        Timer _t;
        uint _videoFrameRate;
        bool _frameToFrame;
};

#endif // __MODULES_IMAGEGENERATOR_VIDEOFILEIMAGEGENERATOR_H__
