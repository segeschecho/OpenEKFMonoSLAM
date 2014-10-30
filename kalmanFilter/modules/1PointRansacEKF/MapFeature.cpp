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

#include "MapFeature.h"

// ------------------------------------------------------------------------------------------------------------

MapFeature::MapFeature()
{
    position = NULL;
    positionDimension = -1;

    covarianceMatrixPos = -1;

    featureType = MAPFEATURE_TYPE_INVALID;

    timesPredicted = 0;
    timesMatched = 0;

#if defined(DEBUG_SHOW_IMAGES)
    lastImagePos = NULL;
#endif
}

// ------------------------------------------------------------------------------------------------------------
MapFeature::MapFeature(const MapFeature &feature)
{
    featureType = feature.featureType;

    // posicion en el espacio
    positionDimension = feature.positionDimension;
    position = new double[positionDimension];

    for (uint i = 0; i < positionDimension; ++i)
    {
        position[i] = feature.position[i];
    }

    // Posicion en la matriz de covarianza.
    covarianceMatrixPos = feature.covarianceMatrixPos;

    // Descriptor del feature
    feature.descriptor.copyTo(descriptor);

    featureType = feature.featureType;

    timesPredicted = feature.timesPredicted;
    timesMatched = feature.timesMatched;

#if defined(DEBUG_SHOW_IMAGES)
    lastImagePos = new double[2];
    lastImagePos[0] = feature.lastImagePos[0];
    lastImagePos[1] = feature.lastImagePos[1];
#endif
}

// ------------------------------------------------------------------------------------------------------------

MapFeature::MapFeature( const double *position, int positionDimension, int covarianceMatrixPos,
                        const cv::Mat &descriptor, MapFeatureType featureType )
{
    this->position = new double[positionDimension];
    this->positionDimension = positionDimension;

    this->covarianceMatrixPos = covarianceMatrixPos;

    descriptor.copyTo(this->descriptor);

    this->featureType = featureType;

    for (int i = 0; i < positionDimension; ++i)
    {
        this->position[i] = position[i];
    }

    timesPredicted = 0;
    timesMatched = 0;

#if defined(DEBUG_SHOW_IMAGES)
    lastImagePos = new double[2];
    lastImagePos[0] = -1.0L;
    lastImagePos[1] = -1.0L;
#endif
}

// ------------------------------------------------------------------------------------------------------------

MapFeature::~MapFeature()
{
    if (position)
    {
        delete [] position;
    }

#if defined(DEBUG_SHOW_IMAGES)
    delete lastImagePos;
#endif
}

// ------------------------------------------------------------------------------------------------------------

std::ostream& operator<<( std::ostream& os, const MapFeature& mapFeature)
{
    if (mapFeature.positionDimension > 0)
    {
        os << mapFeature.position[0];
    }

    for (int i = 1; i < mapFeature.positionDimension; ++i)
    {
        os << ", " << mapFeature.position[i];
    }

    os << " (" << mapFeature.timesMatched << "/" << mapFeature.timesPredicted << ")";

    return os;
}

// ------------------------------------------------------------------------------------------------------------
