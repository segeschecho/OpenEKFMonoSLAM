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

#include <string.h>

#include "../Core/EKFMath.h"
#include "../Configuration/ConfigurationManager.h"

#include "State.h"

// ------------------------------------------------------------------------------------------------------------
// Constructors and Destructors
// ------------------------------------------------------------------------------------------------------------

State::State()
{
    init();
}

// ------------------------------------------------------------------------------------------------------------

State::State(const State& state)
{
    init();

    (*this) = state;
}

// ------------------------------------------------------------------------------------------------------------

State::State(const double *position, const double *orientation, const double *linearVelocity, const double *angularVelocity)
{
    init();

    for (int i = 0; i < 3; ++i)
    {
        this->position[i] = position[i];
        this->linearVelocity[i] = linearVelocity[i];
        this->angularVelocity[i] = angularVelocity[i];
    }

    setOrientation(orientation);
}

// ------------------------------------------------------------------------------------------------------------

State::~State()
{
    if (position)
    {
        delete [] position;
    }

    if (orientation)
    {
        delete [] orientation;
    }

    if (orientationRotationMatrix)
    {
        delete [] orientationRotationMatrix;
    }

    if (linearVelocity)
    {
        delete [] linearVelocity;
    }

    if (angularVelocity)
    {
        delete [] angularVelocity;
    }

    removeAllFeatures();
}

// ------------------------------------------------------------------------------------------------------------
// Public Methods
// ------------------------------------------------------------------------------------------------------------

void State::init()
{
    position = new double[3];
    orientation = new double[4];
    linearVelocity = new double[3];
    angularVelocity = new double[3];

    orientationRotationMatrix = new double[9];

    for (int i = 0; i < 3; ++i)
    {
        position[i] = 0.0L;
        linearVelocity[i] = 0.0L;
        angularVelocity[i] = 0.0L;
    }

    double initOrient[4] = {0};
    setOrientation(initOrient);
}

// ------------------------------------------------------------------------------------------------------------

void State::setOrientation(const double *orientation)
{
    for (int i = 0; i < 4; ++i)
    {
        this->orientation[i] = orientation[i];
    }

    quaternionToRotationMatrix(this->orientation, orientationRotationMatrix);
}

// ------------------------------------------------------------------------------------------------------------

void State::addFeature(MapFeature *mapFeature)
{
    mapFeatures.push_back(mapFeature);

    switch (mapFeature->featureType) {
        case MAPFEATURE_TYPE_DEPTH:
        {
            mapFeaturesDepth.push_back(mapFeature);
        }
        break;

        case MAPFEATURE_TYPE_INVERSE_DEPTH:
        {
            mapFeaturesInvDepth.push_back(mapFeature);
        }
        break;

        default:
        {
            std::cerr << "El feature que se está intentando agregar tiene un tipo inválido." << std::endl;
        }
        break;
    }
}

// ------------------------------------------------------------------------------------------------------------

void State::removeFeature(MapFeature *mapFeatureToRemove)
{
    VectorMapFeature *vectorMapFeatureType = NULL;

    switch (mapFeatureToRemove->featureType) {
        case MAPFEATURE_TYPE_DEPTH:
        {
            vectorMapFeatureType = &mapFeaturesDepth;
        }
        break;

        case MAPFEATURE_TYPE_INVERSE_DEPTH:
        {
            vectorMapFeatureType = &mapFeaturesInvDepth;
        }
        break;

        default:
        {
            std::cerr << "El feature que se está intentando eliminar tiene un tipo inválido (!?)." << std::endl;
        }
        break;
    }

    assert(vectorMapFeatureType);

    searchAndRemove(mapFeatureToRemove, vectorMapFeatureType);
    searchAndRemove(mapFeatureToRemove, &mapFeatures);

    delete mapFeatureToRemove;
}

void State::removeFeatures(const VectorMapFeature &mapFeaturesToRemove)
{
    size_t mapFeaturesToRemoveSize = mapFeaturesToRemove.size();
    for (uint i = 0; i < mapFeaturesToRemoveSize; ++i)
    {
        removeFeature(mapFeaturesToRemove[i]);
    }
}

void State::removeAllFeatures()
{
    // Se realiza delete solamente de los features que estan en mapFeatures, ya que
    // los que estan en mapFeaturesDepth y mapFeaturesInvDepth son los mismos
    size_t mapFeaturesSize = mapFeatures.size();

    for (int i = 0; i < mapFeaturesSize; ++i)
    {
        delete mapFeatures[i];
    }

    mapFeaturesDepth.clear();
    mapFeaturesInvDepth.clear();
    mapFeatures.clear();
}


// ------------------------------------------------------------------------------------------------------------
void State::showDetailed(std::ostream& os) const
{
    os << *this << std::endl;

    // se muestran las posiciones de los features

    os << "Map Features (" << mapFeatures.size() << "):" << std::endl;

    for (unsigned int i = 0; i < mapFeatures.size(); ++i)
    {
        os << i << ": " << *mapFeatures[i] << std::endl;
    }

/*
    os << "features DEPTH (" << mapFeaturesDepth.size() << "):" << std::endl;

    for (uint i = 0; i < mapFeaturesDepth.size(); ++i)
    {
        os << i << ": " << *mapFeaturesDepth[i] << std::endl;
    }

    os << std::endl;

    os << "features INVERSE DEPTH (" << mapFeaturesInvDepth.size() << "):" << std::endl;
    for (uint i = 0; i < mapFeaturesInvDepth.size(); ++i)
    {
        os << i << ": " << *mapFeaturesInvDepth[i] << std::endl;
    }
*/
}

// ------------------------------------------------------------------------------------------------------------

void State::showWithMatlabFormat(std::ostream& os) const
{
    os << "[";
    os << position[0] << ", " << position[1] << ", " << position[2] << ", ";
    os << orientation[0] << ", " << orientation[1] << ", " << orientation[2] << ", " << orientation[3] << ", ";
    os << linearVelocity[0] << ", " << linearVelocity[1] << ", " << linearVelocity[2] << ", ";
    os << angularVelocity[0] << ", " << angularVelocity[1] << ", " << angularVelocity[2];

    for (uint i = 0; i < mapFeatures.size(); ++i)
    {
        MapFeature *currMapFeature = mapFeatures[i];
        for (uint j = 0; j < currMapFeature->positionDimension; ++j)
        {
            os << ", " << currMapFeature->position[j];
        }
    }

    os << "];" << std::endl;
}

// ------------------------------------------------------------------------------------------------------------

State& State::operator=(const State& state)
{
    // Se copia el estado de la camara
    for (uint i = 0; i < 3; ++i)
    {
        position[i] = state.position[i];
        linearVelocity[i] = state.linearVelocity[i];
        angularVelocity[i] = state.angularVelocity[i];
    }

    setOrientation(state.orientation);

    // Elimino todos los features viejos
    removeAllFeatures();

    // Se copia el estado de los features
    size_t mapFeaturesSize = state.mapFeatures.size();

    for (uint i = 0; i < mapFeaturesSize; ++i)
    {
        MapFeature *f = new MapFeature(*state.mapFeatures[i]);
        mapFeatures.push_back(f);

        if (f->featureType == MAPFEATURE_TYPE_DEPTH)
        {
            mapFeaturesDepth.push_back(f);
        }
        else if (f->featureType == MAPFEATURE_TYPE_INVERSE_DEPTH)
        {
            mapFeaturesInvDepth.push_back(f);
        }
    }

    return *this;
}

// ------------------------------------------------------------------------------------------------------------

void State::searchAndRemove(const MapFeature* mapFeatureToRemove, VectorMapFeature* mapFeatures)
{
    VectorMapFeature::iterator itErase = mapFeatures->begin();
    VectorMapFeature::iterator itEnd = mapFeatures->end();

    while(itErase != itEnd && mapFeatureToRemove != *itErase)
    {
        itErase++;
    }

    assert(itErase != itEnd);

    mapFeatures->erase(itErase);
}

// ------------------------------------------------------------------------------------------------------------

void State::write(cv::FileStorage& fs) const
{
    double tempData[13] = {0.0L};

    for (int i = 0; i < 3; ++i)
    {
        tempData[i] = position[i];
        tempData[i + 7] = linearVelocity[i];
        tempData[i + 10] = angularVelocity[i];
    }

    for(int i = 0; i < 4; ++i)
    {
        tempData[i + 3] = orientation[i];
    }

    Matd tempMat(1, 13, tempData);

    fs << tempMat;
    fs << "MapFeaturesInvDepthCount" << static_cast<int>(mapFeaturesInvDepth.size());
    fs << "MapFeaturesDepthCount" << static_cast<int>(mapFeaturesDepth.size());
}

// ------------------------------------------------------------------------------------------------------------

void State::read(const cv::FileNode& node)
{
    assert(false);
}

// ------------------------------------------------------------------------------------------------------------

std::ostream& operator<<( std::ostream& os, const State& state)
{
    double angles[3];
    quaterionToAngles(state.orientation, angles);

    os << "Posicion de la camara: " << state.position[0] << ", "
                                    << state.position[1] << ", "
                                    << state.position[2] << std::endl;

    os << "Orientacion(cuaternions): " << state.orientation[0] << ", "
                                       << state.orientation[1] << ", "
                                       << state.orientation[2] << ", "
                                       << state.orientation[3] << std::endl;

    os << "Orientacion en angulos eulerianos: " << angles[0] << ", "
                                                << angles[1] << ", "
                                                << angles[2] << std::endl;

    os << "Velocidad lineal (con respecto al mundo): " << state.linearVelocity[0] << ", "
                                                       << state.linearVelocity[1] << ", "
                                                       << state.linearVelocity[2] << std::endl;

    os << "Velocidad angular (con respecto a la camara): " << state.angularVelocity[0] << ", "
                                                           << state.angularVelocity[1] << ", "
                                                           << state.angularVelocity[2] << std::endl;

    os << "Cantidad de features en el mapa: " << state.mapFeatures.size() << std::endl;

    return os;
}

// ------------------------------------------------------------------------------------------------------------
