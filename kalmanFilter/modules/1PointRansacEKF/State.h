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

#ifndef __MODULES_1POINTRANSACEKF_STATE_H__
#define __MODULES_1POINTRANSACEKF_STATE_H__

#include <iostream>

#include "../Core/Base.h"
#include "MapFeature.h"

class State
{
public:
    // Constructor and Destructor
    State();
    State(const State&); // constructor por copia
    State(const double *position, const double *orientation, const double *linearVelocity, const double *angularVelocity);
    virtual ~State();

    // Public Methods
    void setOrientation(const double *orientation);

    // IMPORTANTE: las acciones de remover features alteran el featureIndex de los demas features en el mapa
    
    void addFeature(MapFeature *feature);
    void removeFeature(MapFeature *mapFeatureToRemove);
    void removeFeatures(const VectorMapFeature &mapFeaturesToRemove);
    void removeAllFeatures();
    
    void showDetailed(std::ostream& os) const;
    void showWithMatlabFormat(std::ostream& os) const;

    State& operator=(const State& state);
    
    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);

    // Attributes
    double *position;                       //          Position estimate rx, ry, rz
    double *orientation;                    //       Orientation estimate qw, qx, qy, qz
    double *orientationRotationMatrix;      //       Orientation estimate represented as rotation matrix
    double *linearVelocity;                 //   Linear velocity estimate vx, vy, vz
    double *angularVelocity;                //  Angular velocity estimate wx, wy, wz

    VectorMapFeature mapFeaturesDepth;       // Estimated depth features position (R3)
    VectorMapFeature mapFeaturesInvDepth;    // Estimated inverse depth features position (R6)
    VectorMapFeature mapFeatures;            // All features together

private:
    void init();
    void searchAndRemove(const MapFeature* mapFeatureToRemove, VectorMapFeature* mapFeatures);
};

// Show function
std::ostream& operator<<( std::ostream& os, const State& state);

// These write and read functions must be defined for the serialization in FileStorage to work
static void write(cv::FileStorage& fs, const std::string&, const State& state)
{
    state.write(fs);
}

static void read(const cv::FileNode& node, State& state, const State& default_value = State()){
    if(node.empty())
        state = default_value;
    else
        state.read(node);
}

#endif // __MODULES_1POINTRANSACEKF_STATE_H__
