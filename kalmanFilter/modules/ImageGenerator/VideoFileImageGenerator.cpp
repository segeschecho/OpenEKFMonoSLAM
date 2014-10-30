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

#include "VideoFileImageGenerator.h"

// ------------------------------------------------------------------------------------------
// Constructors and Destructor Implementation
// ------------------------------------------------------------------------------------------

VideoFileImageGenerator::VideoFileImageGenerator() {}

// ------------------------------------------------------------------------------------------

VideoFileImageGenerator::VideoFileImageGenerator( std::string pathToVideo, bool frameToFrame) :
                                                 _path(pathToVideo), _frameToFrame(frameToFrame) {}

// ------------------------------------------------------------------------------------------

VideoFileImageGenerator::~VideoFileImageGenerator()
{
    //release video capture
    _captureReference.release();
}


// ------------------------------------------------------------------------------------------
// Inherited Methods Implementation
// ------------------------------------------------------------------------------------------

void VideoFileImageGenerator::init()
{
    // open video file
    _captureReference.open(_path);
    _totalFrames = static_cast<uint>(_captureReference.get(CV_CAP_PROP_FRAME_COUNT));

    _videoFrameRate = static_cast<uint>(_captureReference.get(CV_CAP_PROP_FPS));

    // Se inicia el contador para que tenga tiempo cero
    if (!_frameToFrame)
    {
        _t.start();
        _t.stop();
    }
}

// ------------------------------------------------------------------------------------------

cv::Mat& VideoFileImageGenerator::getNextImage()
{
    uint framesToSetForward;

    if (!_frameToFrame)
    {
         _t.stop();
        double timeElapsed = _t.getElapsedTimeInSec();

        // Se calcula el numero de frames que se tiene que avanzar de acuerdo
        // con el tiempo que paso desde la última llamada
        framesToSetForward = static_cast<uint>( timeElapsed * (static_cast<double>(_videoFrameRate)) );
    }
    else
    {
        framesToSetForward = 1;
    }

    uint actualFrame = static_cast<uint>(_captureReference.get(CV_CAP_PROP_POS_FRAMES)) + framesToSetForward;

    if (!_captureReference.isOpened() || actualFrame >= _totalFrames)
    {
        _image.create(0,0,CV_8U);
        return _image;
    }

    // Se descartan los frames que pasaron en el tiempo entre la ultima vez que se llamo a la funcion y ahora
    _captureReference.set(CV_CAP_PROP_POS_FRAMES, actualFrame);
    _captureReference.read(_image);

    if (!_frameToFrame)
    {
         // Se empieza a contar el tiempo hasta la proxima llamada
        _t.start();
    }

    return _image;
}

// ------------------------------------------------------------------------------------------
