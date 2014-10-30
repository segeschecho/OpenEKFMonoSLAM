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

#ifndef __MODULES_CORE_EKF_MATH_H__
#define __MODULES_CORE_EKF_MATH_H__

#include "Base.h"

#define EPSILON 2.22e-16L
#define DELTA 1.0e-12L
#define PI 3.14159265L
#define CHISQ_95_2 5.9915L
#define CHISQ_99999_2 13.82L

// Pasaje de radianes a grados
#define DEG_TO_RAD(a) (a) * PI / 180.0L
#define RAD_TO_DEG(a) (a) * 180.0L / PI

// Cantidad de 1s en el valor de un binario
static const uchar popCountTable[] =
{
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

// Norma euclideana de R2
double euclideanNorm2(const double *v);

// Norma euclideana de R3
double euclideanNorm3(const double *v);

// Convierte un vector de angulos en R3 a un quaternion
void anglesToQuaternion(const double *v, double *quat);

// Multiplicacion de quaternions
void multiplyQuaternions(const double *q1, const double *q2, double *q);

// Calcula la matriz de rotacion a partir de un cuaternion de orientacion
void quaternionToQuaternionMatrix(const double *q, Matd result);

// Convierte un quaternion en una matriz de rotacion de dimensiones 3x3
void quaternionToRotationMatrix(const double *q, double *rotationMatrix);

// Calcula el vector en XYZ que representa la orientacion del cuaternion pasado por parametros
void makeDirectionalVector(const double theta, const double fi, double *directionalVector);

// Multiplicacion de una matriz de rotacion en R3 por un vector en R3.
// Devuelve un vector en R3.
void multiplyRotationMatrixByVector(const double* rotationMatrix, const double* vector, double* result);

// Suma de matrices
void matrixAdd(const double *v, const double *w, int rows, int cols, double *result);

// Resta de matrices
void matrixSubs(const double *v, const double *w, int rows, int cols, double *result);

// Multiplicacion de matrices
void matrixMult(const double* matrixLeft, int leftRows, int leftCols,
                const double* matrixRight, int rightCols, double* result);

// Rotar un vector de R2 un cierto angulo
void rotateVectorR2(const double *v, double angle, double *rotatedVector);

// Descomposicion de Cholesky de la matriz A.
// Parametros:
// matrix: matriz cuadrada en una sola fila (un arreglo)
// n: dimensiones de la matriz de n x n
// Devuelve:
// l: l*l' = matrix
void cholesky_t(const double *matrix, int n, double *l);

// Conversion de matriz definida positiva a elipse
// matrix: matriz definida positiva (simetrica y cuadrada)
// axes: largo de los ejes de la elipse
// angle: angulo de rotacion del primer eje con respecto al eje X (radianes)
void matrix2x2ToUncertaintyEllipse2D(const Matd &matrix, cv::Size2f &axesSize, double &angle);

// Verifica si un punto esta dentro de una elipse.
bool pointIsInsideEllipse(cv::Point2f point, cv::Point2f ellipseCenter, cv::Size axesSize, double angle);

// convierte un cuaternion a angulos de euler
void quaterionToAngles(const double *q, double *angles);

#endif // __MODULES_CORE_EKF_MATH_H__
