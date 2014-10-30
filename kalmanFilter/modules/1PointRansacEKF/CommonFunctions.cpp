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

#include "../Core/EKFMath.h"
#include "../Configuration/ConfigurationManager.h"

#include "State.h"

// --------------------------------------------------------------------------------------------------------------------

void initState( State &initialState )
{
    initialState.position[0] = 0.0L;
    initialState.position[1] = 0.0L;
    initialState.position[2] = 0.0L;
    
    double orientation[4] = {1.0L, 0.0L, 0.0L, 0.0L};
    initialState.setOrientation(orientation);
    
    initialState.linearVelocity[0] = 0.0L;
    initialState.linearVelocity[1] = 0.0L;
    initialState.linearVelocity[2] = 0.0L;
    
    initialState.angularVelocity[0] = EPSILON;
    initialState.angularVelocity[1] = EPSILON;
    initialState.angularVelocity[2] = EPSILON;
}

// --------------------------------------------------------------------------------------------------------------------

void initCovariance( Matd &initialCovariance )
{
    ExtendedKalmanFilterParameters *ekfParams = ConfigurationManager::getInstance().ekfParams;
    
    // al principio no hay features en el sistema
    Matd(Matd::zeros(13, 13)).copyTo(initialCovariance);
    
    // Se inicia la covarianza para la posicion y orientacion
    for (uint i = 0; i < 7; ++i)
    {
    	initialCovariance.at<double>(i, i) = EPSILON;
    }
    
    // Se inicia la covarianza para la velocidad lineal y angular
    double initialLinearAccelSD2 = ekfParams->initLinearAccelSD * ekfParams->initLinearAccelSD;
    double initialAngularAccelSD2 = ekfParams->initAngularAccelSD * ekfParams->initAngularAccelSD;
    for (uint i = 0; i < 3; ++i)
    {
        initialCovariance.at<double>(i + 7, i + 7) = initialLinearAccelSD2;
        initialCovariance.at<double>(i + 10, i + 10) = initialAngularAccelSD2;
    }
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la matriz de rotacion hecha a partir del cuaternion de orientacion de la camara
// devuelve una matriz de 3x4
// --------------------------------------------------------------------------------------------------------------------

void makeJacobianOfQuaternionToRotationMatrix( const double *quaternion, const double *cameraAxis, double *jacobian )
{
    double q0 = quaternion[0];
    double qx = quaternion[1];
    double qy = quaternion[2];
    double qz = quaternion[3];

    // Matriz de rotacion temporal donde van a estar las derivadas dependiendo de la variable del cuaternion que se esta derivando
    double tempR[9] = {0};
    // matriz temporal donde va a estar el resultado de multiplicar tempR por la posicion de la camara.
    double temp31[3] = {0};

    // Se multiplica por el jacobiando del propio cuaternion que es una matriz diagonal, eso hace que los valores sean los inversos
    // de la columna 2 en adelante de jacobian

    // se llena la primer columna de jacobian (por r del cuaternion)
    tempR[0] =  2*q0; tempR[1] = -2*qz; tempR[2] =  2*qy;
    tempR[3] =  2*qz; tempR[4] =  2*q0; tempR[5] = -2*qx;
    tempR[6] = -2*qy; tempR[7] =  2*qx; tempR[8] =  2*q0;

    multiplyRotationMatrixByVector(tempR, cameraAxis, temp31);
    // el resultado se pone en la primer columna de jacobian
    jacobian[0] = temp31[0];
    jacobian[4] = temp31[1];
    jacobian[8] = temp31[2];

    // se llena la segunda columna de jacobian (por x del cuaternion)
    tempR[0] = 2*qx; tempR[1] =  2*qy; tempR[2] =  2*qz;
    tempR[3] = 2*qy; tempR[4] = -2*qx; tempR[5] = -2*q0;
    tempR[6] = 2*qz; tempR[7] =  2*q0; tempR[8] = -2*qx;

    multiplyRotationMatrixByVector(tempR, cameraAxis, temp31);
    // el resultado se pone en la primer columna de jacobian
    jacobian[1] = temp31[0];
    jacobian[5] = temp31[1];
    jacobian[9] = temp31[2];

    // se llena la tercer columna de jacobian (por y del cuaternion)
    tempR[0] = -2*qy; tempR[1] = 2*qx; tempR[2] =  2*q0;
    tempR[3] =  2*qx; tempR[4] = 2*qy; tempR[5] =  2*qz;
    tempR[6] = -2*q0; tempR[7] = 2*qz; tempR[8] = -2*qy;

    multiplyRotationMatrixByVector(tempR, cameraAxis, temp31);
    // el resultado se pone en la primer columna de jacobian
    jacobian[2] = temp31[0];
    jacobian[6] = temp31[1];
    jacobian[10] = temp31[2];

    // se llena la tercer columna de jacobian (por z del cuaternion)
    tempR[0] = -2*qz; tempR[1] = -2*q0; tempR[2] = 2*qx;
    tempR[3] =  2*q0; tempR[4] = -2*qz; tempR[5] = 2*qy;
    tempR[6] =  2*qx; tempR[7] =  2*qy; tempR[8] = 2*qz;

    multiplyRotationMatrixByVector(tempR, cameraAxis, temp31);
    // el resultado se pone en la primer columna de jacobian
    jacobian[3] = temp31[0];
    jacobian[7] = temp31[1];
    jacobian[11] = temp31[2];
}

// --------------------------------------------------------------------------------------------------------------------

void changeInverseDepthToDepth( const double *inverseDepthPoint, double *depthPoint )
{
    double rho = inverseDepthPoint[5];
    double directionalVector[3] = {0};

    makeDirectionalVector(inverseDepthPoint[3], inverseDepthPoint[4], directionalVector);

    depthPoint[0] = inverseDepthPoint[0] + directionalVector[0]/rho;
    depthPoint[1] = inverseDepthPoint[1] + directionalVector[1]/rho;
    depthPoint[2] = inverseDepthPoint[2] + directionalVector[2]/rho;
}

// --------------------------------------------------------------------------------------------------------------------