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

#include "FileSequenceOnDemandImageGenerator.h"

// ------------------------------------------------------------------------------------------
// Constructors and Destructor Implementation
// ------------------------------------------------------------------------------------------

FileSequenceOnDemandImageGenerator::FileSequenceOnDemandImageGenerator() {}

// ------------------------------------------------------------------------------------------

FileSequenceOnDemandImageGenerator::FileSequenceOnDemandImageGenerator( std::string path, std::string filePrefix, std::string fileExtension,
                                                        int imageBeginIndex, int imageEndIndex, uint frameRate) :
                                                            _path(path), _filePrefix(filePrefix),
                                                            _fileExtension(fileExtension), _imageBeginIndex(imageBeginIndex),
                                                            _imageEndIndex(imageEndIndex), _imageActualIndex(imageBeginIndex),
                                                            _frameRate(frameRate) {}

// ------------------------------------------------------------------------------------------

FileSequenceOnDemandImageGenerator::~FileSequenceOnDemandImageGenerator() {}


// ------------------------------------------------------------------------------------------
// Inherited Methods Implementation
// ------------------------------------------------------------------------------------------

void FileSequenceOnDemandImageGenerator::init() {

   // Se inicia el contador para que tenga tiempo cero
    _t.start();
    _t.stop();
}

// ------------------------------------------------------------------------------------------

cv::Mat& FileSequenceOnDemandImageGenerator::getNextImage()
{
    uint framesToSetForward;

    // move image index to the next one
     _t.stop();
    double timeElapsed = _t.getElapsedTimeInSec();

    // Se calcula el numero de frames que se tiene que avanzar de acuerdo
    // con el tiempo que paso desde la última llamada
    framesToSetForward = static_cast<uint>( timeElapsed * (static_cast<double>(_frameRate)) );

    // if there is not more images, return the valid last one
    if (_imageActualIndex > _imageEndIndex)
    {
        _image.create(0,0,CV_8U);
        return _image;
    }

    // generate complete file name, with path
    size_t lenghImageFullPath = 0;

    //count characters of path and image full name to alloc currentFileName (sprintf need a initialized char pointer)
    lenghImageFullPath += _path.length();
    lenghImageFullPath += _filePrefix.length();
    lenghImageFullPath += _fileExtension.length();
    lenghImageFullPath += 6; // extension point + five zeros of index

    char *currentFileName = new char[lenghImageFullPath + 1];
    sprintf(currentFileName, "%s%s%05d.%s", _path.c_str(), _filePrefix.c_str(), _imageActualIndex, _fileExtension.c_str());

    _image = cv::imread(currentFileName);

    if (!_image.data)
    {
        // TODO: verify if this is correct in the case of a library compilation (for Android)
        std::cerr << "Unable to read image: " << currentFileName << std::endl;
    }

    _imageActualIndex += framesToSetForward;

    _t.start();

    // deallocate pointers
    delete [] currentFileName;

    return _image;
}

// ------------------------------------------------------------------------------------------
