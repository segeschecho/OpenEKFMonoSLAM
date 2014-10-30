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

#include <math.h>
#include "EKFMath.h"

// ------------------------------------------------------------------------------------------------------------

double euclideanNorm3(const double *v)
{
    double vx = v[0];
    double vy = v[1];
    double vz = v[2];

    return sqrt(vx*vx + vy*vy + vz*vz);
}

// ------------------------------------------------------------------------------------------------------------

double euclideanNorm2(const double *v)
{
    double vx = v[0];
    double vy = v[1];

    return sqrt(vx*vx + vy*vy);
}

// ------------------------------------------------------------------------------------------------------------

void anglesToQuaternion(const double *v, double *quat)
{
    double norm = euclideanNorm3(v);

    if (norm < EPSILON)
    {
        quat[0] = 1;
        quat[1] = 0;
        quat[2] = 0;
        quat[3] = 0;
    }
    else
    {
        double normDiv2 = norm/2;
        double sinNormDiv2 = sin(normDiv2);
        quat[0] = cos(normDiv2);
        quat[1] = sinNormDiv2 * (double)(v[0])/norm;
        quat[2] = sinNormDiv2 * (double)(v[1])/norm;
        quat[3] = sinNormDiv2 * (double)(v[2])/norm;
    }
}

// ------------------------------------------------------------------------------------------------------------

void multiplyQuaternions(const double *q1, const double *q2, double *q)
{
    double q1w = q1[0];
    double q1x = q1[1];
    double q1y = q1[2];
    double q1z = q1[3];

    double q2w = q2[0];
    double q2x = q2[1];
    double q2y = q2[2];
    double q2z = q2[3];

    q[0] = q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z;   // w component
    q[1] = q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y;   // x component
    q[2] = q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x;   // y component
    q[3] = q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w;   // z component
}

// ------------------------------------------------------------------------------------------------------------

void quaternionToQuaternionMatrix(const double *q, Matd result)
{
    double qw = q[0];
    double qx = q[1];
    double qy = q[2];
    double qz = q[3];

    cv::Mat( (Matd(4, 4) << qw, -qx, -qy, -qz,
                            qx,  qw, -qz,  qy,
                            qy,  qz,  qw, -qx,
                            qz, -qy,  qx,  qw) ).copyTo(result);
}


// ------------------------------------------------------------------------------------------------------------

void quaternionToRotationMatrix(const double *q, double *rotationMatrix)
{
    // representa la function R=q2r(q) en el código matlab
    double r = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    double r2 = r*r;
    double x2 = x*x;
    double y2 = y*y;
    double z2 = z*z;

    // se usa la matriz de salida como temporal para poder hacer la inversa de esta rotacion
    rotationMatrix[0] = r2 + x2 -y2 -z2;
    rotationMatrix[1] = 2*(x*y - r*z);
    rotationMatrix[2] = 2*(z*x + r*y);
    rotationMatrix[3] = 2*(x*y + r*z);
    rotationMatrix[4] = r2 - x2 + y2 - z2;
    rotationMatrix[5] = 2*(y*z - r*x);
    rotationMatrix[6] = 2*(z*x - r*y);
    rotationMatrix[7] = 2*(y*z + r*x);
    rotationMatrix[8] = r2 - x2 - y2 + z2;
}

// ------------------------------------------------------------------------------------------------------------

void makeDirectionalVector(const double theta, const double fi, double *directionalVector)
{
    double cosfi = cos(fi);

    directionalVector[0] = cosfi * sin(theta);
    directionalVector[1] = -sin(fi);
    directionalVector[2] = cosfi * cos(theta);
}

// ------------------------------------------------------------------------------------------------------------

void multiplyRotationMatrixByVector(const double *rotationMatrix, const double *vector, double *result)
{
    double x = vector[0];
    double y = vector[1];
    double z = vector[2];

    result[0] = rotationMatrix[0]*x + rotationMatrix[1]*y + rotationMatrix[2]*z;
    result[1] = rotationMatrix[3]*x + rotationMatrix[4]*y + rotationMatrix[5]*z;
    result[2] = rotationMatrix[6]*x + rotationMatrix[7]*y + rotationMatrix[8]*z;
}

// ------------------------------------------------------------------------------------------------------------

void matrixAdd(const double *v, const double *w, int rows, int cols, double *result)
{
    int index = 0;
    
    for (int col = 0; col < cols; ++col)
    {
        for (int row = 0; row < rows; ++row)
        {
            index = row * cols + col;
            result[index] = v[index] + w[index];
        }
    }
}

// ------------------------------------------------------------------------------------------------------------

void matrixSubs(const double *v, const double *w, int rows, int cols, double *result)
{
    int index = 0;
    
    for (int col = 0; col < cols; ++col)
    {
        for (int row = 0; row < rows; ++row)
        {
            index = row * cols + col;
            result[index] = v[index] - w[index];
        }
    }
}

// ------------------------------------------------------------------------------------------------------------

void matrixMult(const double* matrixLeft, int leftRows, int leftCols,
                const double* matrixRight, int rightCols, double* result)
{
    for (int i = 0; i < leftRows; ++i)
    {
        for (int j = 0; j < rightCols; ++j)
        {
            result[i * rightCols + j] = 0.0L;
            for (int k = 0; k < leftCols; ++k)
            {
                result[i * rightCols + j] += matrixLeft[i * leftCols + k] * matrixRight[k * rightCols + j];
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------------------

void rotateVectorR2(const double *v, double angle, double *rotatedVector)
{
    double x = v[0];
    double y = v[1];

    rotatedVector[0] = x * cos(angle) + y * (-sin(angle));
    rotatedVector[1] = x * sin(angle) + y * cos(angle);
}

// ------------------------------------------------------------------------------------------------------------

void cholesky_t(const double *matrix, int n, double *l)
{
    double sum1 = 0.0;
    double sum2 = 0.0;
    double sum3 = 0.0;

    l[0] = sqrt(matrix[0]);
    for (int j = 1; j <= n-1; j++)
    {
        l[j*n] = matrix[j*n]/l[0];
    }

    for (int i = 1; i <= (n-2); i++)
    {
        for (int k = 0; k <= (i-1); k++)
        {
            sum1 += l[i*n + k] * l[i*n + k];
        }

        l[i*n + i]= sqrt(matrix[i*n + i] - sum1);
        for (int j = (i+1); j <= (n-1); j++)
        {
            for (int k = 0; k <= (i-1); k++)
            {
                sum2 += l[j*n + k] * l[i*n + k];
            }

            l[j*n + i]= (matrix[j*n + i] - sum2)/l[i*n + i];
        }
    }

    for (int k = 0; k <= (n-2); k++)
    {
        sum3 += l[(n - 1)*n + k] * l[(n - 1)*n + k];
    }

    l[(n-1)*n + n-1] = sqrt(matrix[(n-1)*n + n-1] - sum3);
}

// ------------------------------------------------------------------------------------------------------------

void matrix2x2ToUncertaintyEllipse2D(const Matd &matrix, cv::Size2f &axesSize, double &angle)
{
    double eigenValues[2];
    Matd eigenValuesMat(2, 1, eigenValues);
    Matd eigenVectors;

    cv::eigen(matrix, eigenValuesMat, eigenVectors);

#ifdef DEBUG
    for (int i = 0; i < eigenValuesMat.rows; ++i)
    {
        if (eigenValues[i] <= 0)
        {
            std::cerr << "ERROR: \n " << matrix << std::endl \
                      << "La matriz debe ser definida positiva" << std::endl;
        }

        assert(eigenValues[i] > 0);
    }
#endif

    float sizeAxesX = static_cast<float>( 2.0L * sqrt(eigenValues[0] * CHISQ_95_2) );
    float sizeAxesY = static_cast<float>( 2.0L * sqrt(eigenValues[1] * CHISQ_95_2) );
    axesSize = cv::Size2f(sizeAxesX, sizeAxesY);

    double tan = static_cast<double>(eigenVectors[1][0]) / static_cast<double>(eigenVectors[0][0]);
    angle = atan(tan);
}

// ------------------------------------------------------------------------------------------------------------

bool pointIsInsideEllipse(cv::Point2f point, cv::Point2f ellipseCenter, cv::Size axesSize, double angle)
{
    double majorAxis = MAX(axesSize.width, axesSize.height);
    double minorAxis = MIN(axesSize.width, axesSize.height);

    // Calculamos los puntos Foci

    double f = sqrt(majorAxis * majorAxis - minorAxis * minorAxis);

    double foci1x;
    double foci1y;

    double foci2x;
    double foci2y;

    if (axesSize.height < axesSize.width)
    {
        // Horizontal ellipse
        foci1x = f * cos(angle) + ellipseCenter.x;
        foci1y = f * sin(angle) + ellipseCenter.y;

        foci2x = -f * cos(angle) + ellipseCenter.x;
        foci2y = -f * sin(angle) + ellipseCenter.y;
    }
    else
    {
        // Vertical ellipse
        foci1x = f * (-sin(angle)) + ellipseCenter.x;
        foci1y = f * cos(angle) + ellipseCenter.y;

        foci2x = -f * (-sin(angle)) + ellipseCenter.x;
        foci2y = -f * cos(angle) + ellipseCenter.y;
    }

    // |P - C1| + |P - C2| < 2a
    double pmc1x;
    double pmc1y;
    double pmc2x;
    double pmc2y;

    pmc1x = point.x - foci1x;
    pmc1y = point.y - foci1y;

    pmc2x = point.x - foci2x;
    pmc2y = point.y - foci2y;

    double norm_sum = sqrt(pmc1x*pmc1x + pmc1y*pmc1y) + sqrt(pmc2x*pmc2x + pmc2y*pmc2y);

    return norm_sum <= 2*majorAxis;
}

// ------------------------------------------------------------------------------------------------------------

void quaterionToAngles(const double *q, double *angles)
{
    double q0 = q[0];
    double q1 = q[1];
    double q2 = q[2];
    double q3 = q[3];

    angles[0] = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    angles[1] = asin(2*(q0*q2 - q3*q1));
    angles[2] = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}
