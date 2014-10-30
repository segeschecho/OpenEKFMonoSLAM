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

#include "StateAndCovariancePrediction.h"

#include "../Core/EKFMath.h"
#include "../Configuration/ConfigurationManager.h"

#include "State.h"

// ------------------------------------------------------------------------------------------------------------
//  Fv de la seccion 4.2 del paper 1-POINT (Modelo Dinamico)
// ------------------------------------------------------------------------------------------------------------

void predictState(State &state, double dt)
{
    // Actualizamos r (posicion de la camara con respecto al mundo)
    for (int i = 0; i < 3; ++i)
    {
        state.position[i] += state.linearVelocity[i] * dt;
    }

    // Actualizamos q (quaternion, que indica la orientacion de la camara con respecto al mundo)
    double w[3] = {0};
    for (int i = 0; i < 3; ++i)
    {
        w[i] = state.angularVelocity[i] * dt;
    }

    double q2[4] = {0};
    double q[4] = {0};

    anglesToQuaternion(w, q2);
    multiplyQuaternions(state.orientation, q2, q);

    state.setOrientation(q);
}

// ------------------------------------------------------------------------------------------------------------
// Prediccion de la matriz de covarianza: seccion 4.2 del paper 1-POINT (Modelo Dinamico)
// ------------------------------------------------------------------------------------------------------------

void jacobianDynmodelEq3to7Quat(const State &state, double dt, Matd &result)
{
    double w[3] = {0};
    double q[4] = {0};

    for (int i = 0; i < 3; ++i)
    {
        w[i] = state.angularVelocity[i] * dt;
    }

    anglesToQuaternion(w, q);

    double qw = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    Matd( (Matd(4, 4) << qw, -qx, -qy, -qz,
                         qx,  qw,  qz, -qy,
                         qy, -qz,  qw,  qx,
                         qz,  qy, -qx,  qw) ).copyTo(result);
}

// ------------------------------------------------------------------------------------------------------------
// Auxiliary functions
// Here omegaA is one of omegax, omegay, omegaz; omegaB, omegaC are the other two
// Similarly with qA, qB, qC
// ------------------------------------------------------------------------------------------------------------

inline double derivQuatWByOmegaA(double omegaA, double omega, double dt)
{
    return (-dt / 2.0L) * (omegaA / omega) * sin(omega * dt / 2.0L);
}

// ------------------------------------------------------------------------------------------------------------

inline double derivQuatAByOmegaA(double omegaA, double omega, double dt)
{
    return (dt / 2.0L) * omegaA * omegaA / (omega * omega) * cos(omega * dt / 2.0L)
    + (1.0L / omega) * (1.0L - omegaA * omegaA / (omega * omega)) * sin(omega * dt / 2.0L);
}

// ------------------------------------------------------------------------------------------------------------

inline double derivQuatAByOmegaB(double omegaA, double omegaB, double omega, double dt)
{
    return (omegaA * omegaB / (omega * omega)) * ( (dt / 2.0L) * cos(omega * dt / 2.0L)
                                                  - (1.0L / omega) * sin(omega * dt / 2.0L) );
}

// ------------------------------------------------------------------------------------------------------------

void jacobianDynmodelEq3to7Omega(const State &state, double dt, Matd &result)
{
    double normOmega = euclideanNorm3(&state.angularVelocity[0]);
    double omegaX = state.angularVelocity[0];
    double omegaY = state.angularVelocity[1];
    double omegaZ = state.angularVelocity[2];

    Matd quatMatrix( Matd::zeros(4, 4) );
    quaternionToQuaternionMatrix(state.orientation, quatMatrix);

    Matd derivOmegaDt( (Matd(4, 3) <<
        derivQuatWByOmegaA(omegaX, normOmega, dt),
        derivQuatWByOmegaA(omegaY, normOmega, dt),
        derivQuatWByOmegaA(omegaZ, normOmega, dt),
        derivQuatAByOmegaA(omegaX, normOmega, dt),
        derivQuatAByOmegaB(omegaX, omegaY, normOmega, dt),
        derivQuatAByOmegaB(omegaX, omegaZ, normOmega, dt),
        derivQuatAByOmegaB(omegaY, omegaX, normOmega, dt),
        derivQuatAByOmegaA(omegaY, normOmega, dt),
        derivQuatAByOmegaB(omegaY, omegaZ, normOmega, dt),
        derivQuatAByOmegaB(omegaZ, omegaX, normOmega, dt),
        derivQuatAByOmegaB(omegaZ, omegaY, normOmega, dt),
        derivQuatAByOmegaA(omegaZ, normOmega, dt)) );

    Matd(quatMatrix * derivOmegaDt).copyTo(result);
}

// ------------------------------------------------------------------------------------------------------------
//  Covariance prediction: P = F*P*F' + G*Q*G'
// ------------------------------------------------------------------------------------------------------------

void predictCovariance(Matd &covarianceMatrix, const State &state, double dt)
{
    // Calculamos F
    Matd jacobianF( Matd::eye(13, 13) );

    // columnas:
    // 0  1  2  3  4  5  6  7  8  9  10 11 12
    // rx ry rz qw qx qy qz vx vy vz wx wy wz

    // derivada de las funciones de v_k con respecto a v_k-1
    for (int i = 0; i < 3; ++i)
    {
        jacobianF[i][i + 7] = dt;
    }

    // derivada de las funciones de quaternion con respecto al quaternion
    Matd jacFSubmatrix(jacobianF, cv::Range(3, 7), cv::Range(3, 7));
    jacobianDynmodelEq3to7Quat(state, dt, jacFSubmatrix);

    // derivada de las funciones de quaternion con respecto a la velocidad angular w (omega)
    // Si la velocidad angular (w) es cero, la derivada de las ec 3 a 7 con respecto a w es 0,
    // y la derivada de las ecuaciones 10 a 12 tambien es 0
    if (fabs(state.angularVelocity[0]) < EPSILON &&
        fabs(state.angularVelocity[1]) < EPSILON &&
        fabs(state.angularVelocity[2]) < EPSILON)
    {
        for (int i = 0; i < 3; ++i)
        {
            jacobianF[i + 10][i + 10] = 0;
        }
    }
    else
    {
        jacFSubmatrix = Matd(jacobianF, cv::Range(3, 7), cv::Range(10, 13));
        jacobianDynmodelEq3to7Omega(state, dt, jacFSubmatrix);
    }

    // Calculamos G
    Matd jacobianG( Matd::zeros(13, 6) );

    // columnas:
    //    0    1    2    3    4    5
    // e_vx e_vy e_vz e_wx e_wy e_wz

    for (int i = 0; i < 3; ++i)
    {
        // Derivada de las ecuaciones 7, 8 y 9 con respecto al error de v
        jacobianG[i + 7][i] = 1.0L;

        // Derivada de las ecuaciones 10, 11 y 12 con respecto al error de w
        jacobianG[i + 10][i + 3] = 1.0L;

        // Derivada de las ecuaciones 0, 1 y 2 con respecto al error de v
        jacobianG[i][i] = 1.0L * dt;
    }

    // Derivada de las ecuaciones 4, 5, 6 y 7 con respecto al error de w
    Matd jacGSubmatrix(jacobianG, cv::Range(3, 7), cv::Range(3, 6));
    jacFSubmatrix.copyTo(jacGSubmatrix);

    // Calculamos Q
    ExtendedKalmanFilterParameters *ekfParams = ConfigurationManager::getInstance().ekfParams;
    Matd matrixQ( Matd::zeros(6, 6) );
    double linear_acc_noise = (ekfParams->linearAccelSD * ekfParams->linearAccelSD * dt * dt);
    double angular_acc_noise = (ekfParams->angularAccelSD * ekfParams->angularAccelSD * dt * dt);

    for (int i = 0; i < 3; ++i)
    {
        matrixQ[i][i] = linear_acc_noise;
        matrixQ[i + 3][i + 3] = angular_acc_noise;
    }

    Matd matrix13x13(covarianceMatrix, cv::Range(0, 13), cv::Range(0, 13));
    Matd(jacobianF * matrix13x13 * jacobianF.t() + jacobianG * matrixQ * jacobianG.t()).copyTo(matrix13x13);

    if (covarianceMatrix.cols > 13)
    {
        Matd matrix13xInf(covarianceMatrix, cv::Range(0, 13), cv::Range(13, covarianceMatrix.cols));
        Matd(jacobianF * matrix13xInf).copyTo(matrix13xInf);
    }

    if (covarianceMatrix.rows > 13)
    {
        Matd matrixInfx13(covarianceMatrix, cv::Range(13, covarianceMatrix.rows), cv::Range(0, 13));
        Matd(matrixInfx13 * jacobianF.t()).copyTo(matrixInfx13);
    }
}

// ------------------------------------------------------------------------------------------------------------

void stateAndCovariancePrediction(State &state, Matd &covarianceMatrix)
{
    double dt = 1.0L;

    // FIXME: si quisiesemos optimizar, en predictState se calcula el quaternion resultante de multiplicar
    //        w_c_ck por omega_c (ver seccion 4.2 del paper 1-POINT), y en predictCovariance se recalcula el
    //        mismo resultado. Esto se podria evitar.
    predictCovariance(covarianceMatrix, state, dt);
    predictState(state, dt);
}
