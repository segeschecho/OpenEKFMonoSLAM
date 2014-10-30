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

#include "MeasurementPrediction.h"

#include "../Core/EKFMath.h"
#include "../Core/Timer.h"
#include "../Configuration/ConfigurationManager.h"

#include "State.h"
#include "MapFeature.h"
#include "ImageFeaturePrediction.h"
#include "CommonFunctions.h"

// ------------------------------------------------------------------------------------------------------------
//  Toma un punto proyectado a la camara y lo distorsiona (se hace para no distorsionar toda la imagen)
// ------------------------------------------------------------------------------------------------------------

void distortPoint_matlab(const double* imagePoint, double* imageDistortedPoint)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;
    
    double principalPointX = cameraCalib->cx;
    double principalPointY = cameraCalib->cy;
    double pixelDistX = imagePoint[0] - principalPointX;
    double pixelDistY = imagePoint[1] - principalPointY;
    
    double pointToPrincipalPointDistX = cameraCalib->dx * pixelDistX;  //en mm
    double pointToPrincipalPointDistY = cameraCalib->dy * pixelDistY;  //en mm
    
    double distPow2 =  pointToPrincipalPointDistX * pointToPrincipalPointDistX +
    pointToPrincipalPointDistY * pointToPrincipalPointDistY; //en mm
    
    double ru = sqrt(distPow2);
    double radialDistortion = ru / (1.0L + cameraCalib->k1 * distPow2 + cameraCalib->k2 * distPow2 * distPow2);
    
    for (int k = 0; k < 10; ++k)
    {
        double radDist2 = radialDistortion * radialDistortion;
        double radDist3 = radDist2 * radialDistortion;
        double radDist4 = radDist2 * radDist2;
        double radDist5 = radDist4 * radialDistortion;
        
        double f = radialDistortion + cameraCalib->k1 * radDist3 + cameraCalib->k2 * radDist5 - ru;
        double fp = 1 + 3*cameraCalib->k1*radDist2 + 5*cameraCalib->k2*radDist4;
        radialDistortion = radialDistortion - f / fp;
    }
    
    double rd2 = radialDistortion * radialDistortion;
    double rd4 = rd2 * rd2;
    double d = (1.0L + cameraCalib->k1 * rd2 + cameraCalib->k2 * rd4);
    
    imageDistortedPoint[0] = principalPointX + pixelDistX / d;
    imageDistortedPoint[1] = principalPointY + pixelDistY / d;
}

void distortPoint(const double* imagePoint, double* imageDistortedPoint)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    double principalPointX = cameraCalib->cx;
    double principalPointY = cameraCalib->cy;
    double pixelDistX = imagePoint[0] - principalPointX;
    double pixelDistY = imagePoint[1] - principalPointY;

    double pointToPrincipalPointDistX = cameraCalib->dx * pixelDistX;  //en mm
    double pointToPrincipalPointDistY = cameraCalib->dy * pixelDistY;  //en mm

    double distPow2 =  pointToPrincipalPointDistX * pointToPrincipalPointDistX +
                       pointToPrincipalPointDistY * pointToPrincipalPointDistY; //en mm

    double radialDistortion = 1.0L + cameraCalib->k1 * distPow2 + cameraCalib->k2 * distPow2 * distPow2;

    imageDistortedPoint[0] = principalPointX + pixelDistX / radialDistortion;
    imageDistortedPoint[1] = principalPointY + pixelDistY / radialDistortion;
}

// ------------------------------------------------------------------------------------------------------------
//  Proyecta un punto (xyz) al plano de la camara y obtiene sus coordenadas en la imagen
// ------------------------------------------------------------------------------------------------------------

void projectToCameraFrame(const double* point, double* imageCoordinates)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    double pointX = point[0];
    double pointY = point[1];
    double pointZ = point[2];

    imageCoordinates[0] = cameraCalib->cx + (cameraCalib->fx * pointX / pointZ);
    imageCoordinates[1] = cameraCalib->cy + (cameraCalib->fy * pointY / pointZ);
}

// ------------------------------------------------------------------------------------------------------------
//  Recibe un punto en InverseDepth (x,y,z,theta,fi,rho) y el punto de la camara, todo con respecto al mundo.
//  Devuelve las coordenadas del punto con respecto al eje de la camara (en xyz).
// ------------------------------------------------------------------------------------------------------------

void changeToCameraReferenceAxisInverseDepth(const double* point, const double* cameraPosition, const double* rotationMatrix, double* pointInCameraAxis)
{
    double rho = point[5];
    double directionalVector[3] = {0};
    double pointAtCameraAxis[3] = {0};

    makeDirectionalVector(point[3], point[4], directionalVector);

    pointAtCameraAxis[0] = rho*( point[0] - cameraPosition[0] ) + directionalVector[0];
    pointAtCameraAxis[1] = rho*( point[1] - cameraPosition[1] ) + directionalVector[1];
    pointAtCameraAxis[2] = rho*( point[2] - cameraPosition[2] ) + directionalVector[2];

    multiplyRotationMatrixByVector(rotationMatrix, pointAtCameraAxis, pointInCameraAxis);
}

// ------------------------------------------------------------------------------------------------------------
//  Recibe un punto en el mundo (xyz) y el punto (xyz) de la camara. Todo con respecto al mundo.
//  Devuelve la posicion del primer punto con respecto al eje de la camara.
// ------------------------------------------------------------------------------------------------------------

void changeToCameraReferenceAxis(const double* point, const double* cameraPosition, const double* rotationMatrix, double* pointInCameraAxis)
{
    double pointAtCameraAxis[3] = {0};
    pointAtCameraAxis[0] = point[0] - cameraPosition[0];
    pointAtCameraAxis[1] = point[1] - cameraPosition[1];
    pointAtCameraAxis[2] = point[2] - cameraPosition[2];

    // Multiplicacion de la matriz por el vector
    multiplyRotationMatrixByVector(rotationMatrix, pointAtCameraAxis, pointInCameraAxis);
}

// ------------------------------------------------------------------------------------------------------------
//  indica si un punto en el espacio (en coordenadas con respecto al eje de la camara),
//  se encuentra frente a la misma.
// ------------------------------------------------------------------------------------------------------------
bool isInFrontOfCamera(double *feature)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    double atanxz = RAD_TO_DEG(atan2(feature[0], feature[2]));
    double atanyz = RAD_TO_DEG(atan2(feature[1], feature[2]));

    return -cameraCalib->angularVisionX < atanxz && atanxz < cameraCalib->angularVisionX &&
           -cameraCalib->angularVisionY < atanyz && atanyz < cameraCalib->angularVisionY;
}

// ------------------------------------------------------------------------------------------------------------
//  indica si un punto proyectado al frame de la camara se encuentra dentro de los limites de la mismo
// ------------------------------------------------------------------------------------------------------------
bool isVisibleInImageFrame(double *imagePoint)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    return (imagePoint[0] > 0 && imagePoint[0] < cameraCalib->pixelsX && imagePoint[1] > 0 && imagePoint[1] < cameraCalib->pixelsY);
}


// ------------------------------------------------------------------------------------------------------------
// Toma los features que se encuentran en el mapa y por cada uno, los proyecta al plano de la camara
// (segun la posicion y rotación predicha de la misma), le agrega distorsion (dado que los puntos en el mapa
// estan des-distorsionados) y devuelve la posición en donde deverían estar en la siguiente imagen.
// ------------------------------------------------------------------------------------------------------------
void predictFeatures( const State& state,
                      const VectorMapFeature &features,
                      const std::vector<int> &featureIndexes,
                      VectorImageFeaturePrediction &predictedFeatures )
{

}


// ------------------------------------------------------------------------------------------------------------
// Toma los features que se encuentran en el mapa y por cada uno, los proyecta al plano de la camara
// (segun la posicion y rotación predicha de la misma), le agrega distorsion (dado que los puntos en el mapa
// estan des-distorsionados) y devuelve la posición en donde deverían estar en la siguiente imagen.
// ------------------------------------------------------------------------------------------------------------
void predictMeasurementState( const State &state, const VectorMapFeature &features, const std::vector<int> &featureIndexes,
                              VectorImageFeaturePrediction &predictedFeatures, VectorMapFeature &notPredictedFeatures )
{
    size_t countFeatures = features.size();
    double featureInCameraAxis[3] = {0};
    double featureProjection[2] = {0};
    
    if (countFeatures == 0)
    {
        return;
    }
    
    Matd rotationMatrix(3, 3, state.orientationRotationMatrix);
    Matd inverseRotationMatrix(rotationMatrix.inv());
    Matd transposeRotationMatrix(rotationMatrix.t());
    
    // Para cada feature se hace su prediccion
    for (int i = 0; i < countFeatures; ++i)
    {
        MapFeature *f = features[i];
        
        if (f->featureType == MAPFEATURE_TYPE_INVERSE_DEPTH)
        {
            changeToCameraReferenceAxisInverseDepth(f->position, state.position, transposeRotationMatrix.ptr<double>(), featureInCameraAxis);
        }
        else
        {
            changeToCameraReferenceAxis(f->position, state.position, inverseRotationMatrix.ptr<double>(), featureInCameraAxis);
        }
        
        bool featurePredicted = false;
        if (isInFrontOfCamera(featureInCameraAxis))
        {
            projectToCameraFrame(featureInCameraAxis, featureProjection);
            distortPoint_matlab(featureProjection, featureProjection);
            
            if (isVisibleInImageFrame(featureProjection))
            {
                int featureIndex;
                
                if (featureIndexes.size() == 0)
                {
                    featureIndex = i;
                }
                else
                {
                    featureIndex = featureIndexes[i];
                }
                
                // Se inicializa un feature prediction con el orden que tiene el feature en el arreglo dentro del estado
                // La matriz de covarianza de la prediccion del feature se inicializa luego en el algoritmo
                ImageFeaturePrediction *featurePrediction = new ImageFeaturePrediction(featureIndex, featureProjection);
                predictedFeatures.push_back(featurePrediction);
                featurePredicted = true;
            }
        }
        
        if (!featurePredicted)
        {
            notPredictedFeatures.push_back(f);
        }
    }
}


// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la funcion que proyecta los puntos en 3D al plano de la imagen de la camara
// la matriz es de 2x3 porque se derivan las 2 funciones de proyeccion por las variables X, Y, Z.
// Resultado 2x3
// --------------------------------------------------------------------------------------------------------------------
void makeJacobianOfFrameProjectionFunction(const State &state, const double *inverseRotationMatrix, const double *pointXYZ, bool isInverseDepth, double *jacobian)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    double fdx = cameraCalib->fx;
    double fdy = cameraCalib->fy;

    double pointInCameraAxis[3] = {0};

    if (isInverseDepth)
    {
        changeToCameraReferenceAxisInverseDepth(pointXYZ, state.position, inverseRotationMatrix, pointInCameraAxis);
    }
    else
    {
        changeToCameraReferenceAxis(pointXYZ, state.position, inverseRotationMatrix, pointInCameraAxis);
    }

    jacobian[0] = fdx/pointInCameraAxis[2];
    jacobian[1] = 0;
    jacobian[2] = -pointInCameraAxis[0] * fdx / (pointInCameraAxis[2] * pointInCameraAxis[2]);
    jacobian[3] = 0;
    jacobian[4] = fdy / pointInCameraAxis[2];
    jacobian[5] = -pointInCameraAxis[1] * fdy / (pointInCameraAxis[2] * pointInCameraAxis[2]);
}



// --------------------------------------------------------------------------------------------------------------------
// Realiza el calculo del jacobiano para las funciones que desdistorsionan (radialmente) el punto proyectado en la imagen.
// se deriva con respecto a las coordenadas distorcionadas.
// Nota: Hay que hacer la inversa de este jacobiano si se quiere el de las funciones que distorsionan
// (es decir, las funciones que resultan de despejar las coordenadas distorsionadas)
// la matriz de retorno es de 2x2 porque se derivan las 2 funciones de des-distorsion por cada coordenada distorsionada
// --------------------------------------------------------------------------------------------------------------------
void makeJacobianOfDistortionFunction(const double *distortedImagePoint, double *jacobian)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    double pixelDistX = distortedImagePoint[0] - cameraCalib->cx;
    double pixelDistY = distortedImagePoint[1] - cameraCalib->cy;

    double pointToPrincipalPointDistX = cameraCalib->dx * (pixelDistX);  //en mm
    double pointToPrincipalPointDistY = cameraCalib->dy * (pixelDistY);  //en mm

    double distPow2 =  pointToPrincipalPointDistX * pointToPrincipalPointDistX +
                            pointToPrincipalPointDistY * pointToPrincipalPointDistY; //en mm

    double radialDistortion = 1 + cameraCalib->k1 * distPow2 + cameraCalib->k2 * distPow2 * distPow2;

    // Derivada de la funcion de distorsion en X por la coordenada distorsionada en X
    double dRadialDistXForDistImageX = (radialDistortion) + pixelDistX * (cameraCalib->k1 + 2*cameraCalib->k2*distPow2) * (2*pixelDistX*cameraCalib->dx*cameraCalib->dx);
    // Derivada de la funcion de distorsion en Y por la coordenada distorsionada en Y
    double dRadialDistYForDistImageY = (radialDistortion) + pixelDistY * (cameraCalib->k1 + 2*cameraCalib->k2*distPow2) * (2*pixelDistY*cameraCalib->dy*cameraCalib->dy);

    // Derivada de la funcion de distorsion en X por la coordenada distorsionada en Y
    double dRadialDistXForDistImageY = (pixelDistX) * (cameraCalib->k1 + 2*cameraCalib->k2*distPow2) * (2*(pixelDistY)*cameraCalib->dy*cameraCalib->dy);
    // Derivada de la funcion de distorsion en Y por la coordenada distorsionada en X
    double dRadialDistYForDistImageX = (pixelDistY) * (cameraCalib->k1 + 2*cameraCalib->k2*distPow2) * (2*(pixelDistX)*cameraCalib->dx*cameraCalib->dx);

    jacobian[0] = dRadialDistXForDistImageX;
    jacobian[1] = dRadialDistXForDistImageY;
    jacobian[2] = dRadialDistYForDistImageX;
    jacobian[3] = dRadialDistYForDistImageY;
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la composicion entre la funcion de distorsion y la de proyeccion en la imagen
// la matriz resultante es de 2x3
// --------------------------------------------------------------------------------------------------------------------
void makeJacobianOfProjection(const State &state, const double *inverseRotationMatrix, const double *distortedImagePoint, const double *worldPoint, bool isInverseDepth, double *jacobian)
{
    double dj[4] = {0};      // distortionJacobian
    double fpj[6] = {0};     // frameProjectionJacobian
    double *idj = NULL;

    makeJacobianOfDistortionFunction(distortedImagePoint, dj);
    makeJacobianOfFrameProjectionFunction(state, inverseRotationMatrix, worldPoint, isInverseDepth, fpj);

    Matd idjMatrix(2, 2, dj);
    Matd idjMatrixInv(idjMatrix.inv());    // inverse distortion jacobian
    idj = idjMatrixInv.ptr<double>();

    jacobian[0] = idj[0] * fpj[0] + idj[1] * fpj[3];
    jacobian[1] = idj[0] * fpj[1] + idj[1] * fpj[4];
    jacobian[2] = idj[0] * fpj[2] + idj[1] * fpj[5];
    jacobian[3] = idj[2] * fpj[0] + idj[3] * fpj[3];
    jacobian[4] = idj[2] * fpj[1] + idj[3] * fpj[4];
    jacobian[5] = idj[2] * fpj[2] + idj[3] * fpj[5];
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la parte derecha de la formula que pasa un punto en el eje de coordenadas del mundo al eje
// de coordenadas de la camara. el jacobiano de salida es de 3x3
// Nota: La parte derecha es -R_w * r_C de la funcion distribuida R_w*(y_i - r_C)
// --------------------------------------------------------------------------------------------------------------------
void makeJacobianOfChangeToCameraAxisRightPart(const double *inverseRotationMatrix, double *jacobian)
{
    jacobian[0] = -inverseRotationMatrix[0];
    jacobian[2] = -inverseRotationMatrix[1];
    jacobian[2] = -inverseRotationMatrix[2];
    jacobian[3] = -inverseRotationMatrix[3];
    jacobian[4] = -inverseRotationMatrix[4];
    jacobian[5] = -inverseRotationMatrix[5];
    jacobian[6] = -inverseRotationMatrix[6];
    jacobian[7] = -inverseRotationMatrix[7];
    jacobian[8] = -inverseRotationMatrix[8];
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la parte derecha de la formula que pasa un punto en el eje de coordenadas del mundo al eje
// de coordenadas de la camara. el jacobiano de salida es de 3x3
// Nota: La parte derecha es -R_w * r_C de la funcion distribuida R_w*(y_i - r_C)
// Se usa para inverseDepth
// --------------------------------------------------------------------------------------------------------------------
void makeJacobianOfChangeToCameraAxisRightPart(const double *inverseRotationMatrix, const double rho, double *jacobian)
{
    makeJacobianOfChangeToCameraAxisRightPart(inverseRotationMatrix, jacobian);

    jacobian[0] *= rho;
    jacobian[2] *= rho;
    jacobian[2] *= rho;
    jacobian[3] *= rho;
    jacobian[4] *= rho;
    jacobian[5] *= rho;
    jacobian[6] *= rho;
    jacobian[7] *= rho;
    jacobian[8] *= rho;
}


// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la composicion de funciones entre el cambio al eje de la camara, la proyeccion al frame y
// y el agregado de distorsion
// Todo derivado por el estado de la camara (posicion, orientacion, velocidad lineal, velocidad angular)
// el resultado es una matriz de 2x3
// --------------------------------------------------------------------------------------------------------------------

void makeJacobianOfProjectionAndChangeToCameraAxis( const State &state,
                                                    const double *inverseRotationMatrix,
                                                    const double *distortedImagePoint,
                                                    const double *worldPoint,
                                                    bool isInverseDepth,
                                                    Matd &jacobian )
{
    double carpJacobian[9] = {0};  //ChangeToCameraAxisRightPart jacobian 3x3
    double pJacobian[6] = {0};     //Projection jacobian 2x3

    if (isInverseDepth)
    {
        makeJacobianOfChangeToCameraAxisRightPart(inverseRotationMatrix, worldPoint[5], carpJacobian);
    }
    else
    {
        makeJacobianOfChangeToCameraAxisRightPart(inverseRotationMatrix, carpJacobian);
    }

    makeJacobianOfProjection(state, inverseRotationMatrix, distortedImagePoint, worldPoint, isInverseDepth, pJacobian);

    Matd carpJacobianMatrix(3, 3, carpJacobian);
    Matd pJacobianMatrix(2, 3, pJacobian);

    Matd(pJacobianMatrix * carpJacobianMatrix).copyTo(jacobian);
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la funcion que cambia el punto xyz al eje de la camara, derivado por el cuaternion
// Matriz de 2x4
// --------------------------------------------------------------------------------------------------------------------
void makeJacobianOfChangeToCameraAxisByQuaternion( const State &state,
                                                   const double *inverseRotationMatrix,
                                                   const double *distortedImagePoint,
                                                   const double *worldPoint,
                                                   bool isInverseDepth,
                                                   Matd &jacobian )
{
    double pJacobian[6]           = {0};      // matriz de 2x3
    double qtrmJacobian[12]       = {0};      // matriz de 3x4
    double quaternionConjugate[4] = {state.orientation[0], -state.orientation[1], -state.orientation[2], -state.orientation[3]};
    double cameraAxis[3]          = {worldPoint[0] - state.position[0], worldPoint[1] - state.position[1], worldPoint[2] - state.position[2]};

    if (isInverseDepth)
    {
        double directionalVector[3] = {0};
        double rho = worldPoint[5];

        makeDirectionalVector(worldPoint[3], worldPoint[4], directionalVector);

        cameraAxis[0] = cameraAxis[0] * rho + directionalVector[0];
        cameraAxis[1] = cameraAxis[1] * rho + directionalVector[1];
        cameraAxis[2] = cameraAxis[2] * rho + directionalVector[2];
    }

    makeJacobianOfQuaternionToRotationMatrix(quaternionConjugate, cameraAxis, qtrmJacobian);
    // Se multiplica por diag([1 -1 -1 -1])
    qtrmJacobian[1] = -qtrmJacobian[1];
    qtrmJacobian[5] = -qtrmJacobian[5];
    qtrmJacobian[9] = -qtrmJacobian[9];
    qtrmJacobian[2] = -qtrmJacobian[2];
    qtrmJacobian[6] = -qtrmJacobian[6];
    qtrmJacobian[10] = -qtrmJacobian[10];
    qtrmJacobian[3] = -qtrmJacobian[3];
    qtrmJacobian[7] = -qtrmJacobian[7];
    qtrmJacobian[11] = -qtrmJacobian[11];

    makeJacobianOfProjection(state, inverseRotationMatrix, distortedImagePoint, worldPoint, isInverseDepth, pJacobian);

    Matd pJacobianMatrix(2, 3, pJacobian);
    Matd qtrmJacobianMatrix(3, 4, qtrmJacobian);

    Matd(pJacobianMatrix * qtrmJacobianMatrix).copyTo(jacobian);
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiando de medicion (composicion de funciones de pasaje a eje de la camara, proyeccion a frame y distorsion)
// derivado por el estado (posicion, orientacion, velocidad linea, angular)
// matriz de salida 2x13
// --------------------------------------------------------------------------------------------------------------------
void makeJacobianOfMeasurementByState( const State &state,
                                       const double *inverseRotationMatrix,
                                       const double *distortedImagePoint,
                                       const double *worldPoint,
                                       bool isInverseDepth,
                                       Matd &jacobian )
{
    Matd measurementByPositionJacobian(jacobian, cv::Range(0, 2), cv::Range(0, 3));
    Matd measurementByQuaternionJacobian(jacobian, cv::Range(0, 2), cv::Range(3, 7));

    makeJacobianOfProjectionAndChangeToCameraAxis(state, inverseRotationMatrix, distortedImagePoint, worldPoint, isInverseDepth, measurementByPositionJacobian);
    makeJacobianOfChangeToCameraAxisByQuaternion(state, inverseRotationMatrix, distortedImagePoint, worldPoint, isInverseDepth, measurementByQuaternionJacobian);
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano del modelo de medicion con respecto al feature que se le pase actualmente
// resultado 2x3
// --------------------------------------------------------------------------------------------------------------------

void makeJacobianOfMeasurementByFeatureiDepth( const State &state,
                                               double *inverseRotationMatrix,
                                               const double *distortedImagePoint,
                                               const double *worldPoint,
                                               Matd &jacobian )
{
    double pJacobian[6] = {0};  // matriz de 2x3
    makeJacobianOfProjection(state, inverseRotationMatrix, distortedImagePoint, worldPoint, false, pJacobian);

    // lo declaro aca para no tener que hacer el .clone que es mas lento
    Matd pJacobianMatrix(2, 3, pJacobian);
    Matd rotation(3, 3, inverseRotationMatrix);
    Matd(pJacobianMatrix * rotation).copyTo(jacobian);
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano del modelo de medicion con respecto al feature que se le pase actualmente
// resultado 2x3
// --------------------------------------------------------------------------------------------------------------------

void makeJacobianOfMeasurementByFeatureiInverseDepth( const State &state,
                                                      const double *inverseRotationMatrix,
                                                      const double *distortedImagePoint,
                                                      const double *worldPoint,
                                                      Matd &jacobian)
{
    // worldPoint es (x, y, z, theta, phi, rho)
    double theta = worldPoint[3];
    double phi = worldPoint[4];
    double rho = worldPoint[5];
    double pJacobian[6] = {0};  // matriz de 2x3

    double cosphi = cos(phi);
    double costheta = cos(theta);
    double sintheta = sin(theta);
    double sinphi = sin(phi);
    double directionalVectorByThetaDerivate[3] = {cosphi * costheta, 0 , -cosphi * sintheta};
    double directionalVectorByPhiDerivate[3]   = {-sinphi * sintheta, -cosphi, -sinphi * costheta};

    double pointInCameraAxis[3] = {worldPoint[0] - state.position[0], worldPoint[1] - state.position[1], worldPoint[2] - state.position[2]};

    double rotationByThetaDerivate[3] = {0};
    double rotationByPhiDerivate[3] = {0};
    double rotationByPointInCameraAxis[3] = {0};
    double rotationByRho[9] = {0};

    // se calcula el jacobiano con respecto al feature: devuelve matriz de 3x6
    multiplyRotationMatrixByVector(inverseRotationMatrix, directionalVectorByThetaDerivate, rotationByThetaDerivate);
    multiplyRotationMatrixByVector(inverseRotationMatrix, directionalVectorByPhiDerivate, rotationByPhiDerivate);
    multiplyRotationMatrixByVector(inverseRotationMatrix, pointInCameraAxis, rotationByPointInCameraAxis);
    rotationByRho[0] = rho * inverseRotationMatrix[0];
    rotationByRho[1] = rho * inverseRotationMatrix[1];
    rotationByRho[2] = rho * inverseRotationMatrix[2];
    rotationByRho[3] = rho * inverseRotationMatrix[3];
    rotationByRho[4] = rho * inverseRotationMatrix[4];
    rotationByRho[5] = rho * inverseRotationMatrix[5];
    rotationByRho[6] = rho * inverseRotationMatrix[6];
    rotationByRho[7] = rho * inverseRotationMatrix[7];
    rotationByRho[8] = rho * inverseRotationMatrix[8];

    // creo un Matd con las multiplicaciones anteriores
    Matd featureDirectionalVectorJacobian(3, 6);
    for (uint i = 0; i < 3; ++i)
    {
        Matd featureDirectionalVectorJacobianRowi( (Matd(1, 6) << rotationByRho[0 + 3*i],
                                                                  rotationByRho[1 + 3*i],
                                                                  rotationByRho[2 + 3*i],
                                                                  rotationByThetaDerivate[i],
                                                                  rotationByPhiDerivate[i],
                                                                  pointInCameraAxis[i]) );

        featureDirectionalVectorJacobianRowi.copyTo(featureDirectionalVectorJacobian.row(i));
    }

    // se calcula el jacobiano del la funcion de proyeccion
    makeJacobianOfProjection(state, inverseRotationMatrix, distortedImagePoint, worldPoint, true, pJacobian);
    // lo declaro aca para no tener que hacer el .clone que es mas lento
    Matd pJacobianMatrix(2, 3, pJacobian);
    Matd(pJacobianMatrix * featureDirectionalVectorJacobian).copyTo(jacobian);
}

// --------------------------------------------------------------------------------------------------------------------
// Calcula la matriz de covarianza para el punto que se le pasa (ya sea depth o inverse depth)
// devuelve una matriz de 2x2
// --------------------------------------------------------------------------------------------------------------------
void makeMeasurementCovariance( const State &state,
                                double *inverseRotationMatrix,
                                const MapFeature *worldPoint,
                                const Matd &predictedStateCovariance,
                                const Matd &stateCovarianceSubMatrix,
                                ImageFeaturePrediction *distortedImagePoint,
                                Matd &jacobian )
{
    Matd hiByStateJacobian( Matd::zeros(2, 13) );

    int dimension = worldPoint->positionDimension;
    int covariancePosition = worldPoint->covarianceMatrixPos;
    bool isInverseDepth = (worldPoint->featureType == MAPFEATURE_TYPE_INVERSE_DEPTH);

    makeJacobianOfMeasurementByState( state,
                                      inverseRotationMatrix,
                                      distortedImagePoint->imagePos,
                                      worldPoint->position,
                                      isInverseDepth,
                                      hiByStateJacobian );

    Matd hiByFeatureJacobian( Matd::zeros(2, dimension) );

    // devuelve una matriz de 2xdimension
    if(worldPoint->featureType == MAPFEATURE_TYPE_INVERSE_DEPTH)
    {
        makeJacobianOfMeasurementByFeatureiInverseDepth( state,
                                                         inverseRotationMatrix,
                                                         distortedImagePoint->imagePos,
                                                         worldPoint->position,
                                                         hiByFeatureJacobian );
    }
    else
    {
        makeJacobianOfMeasurementByFeatureiDepth( state,
                                                  inverseRotationMatrix,
                                                  distortedImagePoint->imagePos,
                                                  worldPoint->position,
                                                  hiByFeatureJacobian );
    }

    Matd featureiCovarianceSubMatrix( predictedStateCovariance,
                                      cv::Range(covariancePosition, covariancePosition + dimension),
                                      cv::Range(0, predictedStateCovariance.cols) );

    Matd hiByP( Matd::zeros(2, predictedStateCovariance.cols) );
    // se multiplica el jacobiano del feature por las filas de la covarianza asociada al feature i
    //     2x3 * 3x(13 + 3*depths + 6*inverseDepths) +  2x13 * 13x(13 + 3*depths + 6*inverseDepths)  Si es Depth
    // resultado de 2x(13 + 3*depths + 6*inverseDepths)
    Matd(hiByFeatureJacobian * featureiCovarianceSubMatrix + hiByStateJacobian * stateCovarianceSubMatrix).copyTo(hiByP);

    //
    // Ri se define como la identidad de 2x2
    //
    // se calcula (HiByP)*Hi' + Ri
    // resultado de 2x2
    Matd(hiByP(cv::Range(0, 2), cv::Range(0, 13)) * hiByStateJacobian.t() +
         hiByP(cv::Range(0, 2), cv::Range(covariancePosition, covariancePosition + dimension)) * hiByFeatureJacobian.t() +
         Matd::eye(2,2) ).copyTo(distortedImagePoint->covarianceMatrix);

    // se crea la matriz completa que representa el jacobiano (dado que está por partes para hacer las cuentas)
    hiByStateJacobian.copyTo( jacobian(cv::Range(0, 2), cv::Range(0, 13)) );
    hiByFeatureJacobian.copyTo( jacobian(cv::Range(0, 2), cv::Range(covariancePosition, covariancePosition + dimension)) );
}

// --------------------------------------------------------------------------------------------------------------------
// Realiza el calculo de la matriz de covarianza para todas las mediciones que se predijeron usando predictMeasurementState
// resutado de 2x2
// lo que se quiere hacer es Hi * P * Hi' + Ri para cada una de las mediciones
// --------------------------------------------------------------------------------------------------------------------

void predictMeasurementCovariance( const State &state,
                                   const Matd &predictedStateCovariance,
                                   VectorImageFeaturePrediction &predictedDistortedFeatures,
                                   VectorMatd &predictedFeatureJacobians )
{
    Matd rotationMatrix(3, 3, state.orientationRotationMatrix);
    Matd inverseRotationMatrix(rotationMatrix.inv());
    
    Matd stateCovarianceSubMatrix( predictedStateCovariance, cv::Range(0, 13), cv::Range(0, predictedStateCovariance.cols) );

    ImageFeaturePrediction *distortedImagePoint = NULL;
    MapFeature *worldPoint = NULL;

    int stateMapFeaturesDepthSize = static_cast<int>(state.mapFeaturesDepth.size());
    int stateMapFeaturesInvDepthSize = static_cast<int>(state.mapFeaturesInvDepth.size());
    for (uint i = 0; i < predictedDistortedFeatures.size(); ++i)
    {
        distortedImagePoint = predictedDistortedFeatures[i];

        worldPoint = state.mapFeatures[distortedImagePoint->featureIndex];

        // donde se va a guardar el jacobiano de cada feature.
        Matd *jacobian = new Matd( Matd::zeros(2, 13 + 3*stateMapFeaturesDepthSize + 6*stateMapFeaturesInvDepthSize) );
        
        makeMeasurementCovariance( state,
                                   inverseRotationMatrix.ptr<double>(),
                                   worldPoint,
                                   predictedStateCovariance,
                                   stateCovarianceSubMatrix,
                                   distortedImagePoint,
                                   *jacobian );

        predictedFeatureJacobians.push_back(jacobian);
    }
}


// ------------------------------------------------------------------------------------------------------------

void predictCameraMeasurements( const State &state,
                                const Matd &predictedStateCovariance,
                                const VectorMapFeature &features,
                                const std::vector<int> &featureIndexes,
                                VectorImageFeaturePrediction &predictedDistortedFeatures,
                                VectorMatd &predictedFeatureJacobians,
                                VectorMapFeature &notPredictedFeatures)
{
    predictMeasurementState(state, features, featureIndexes, predictedDistortedFeatures, notPredictedFeatures);

    predictMeasurementCovariance( state,
                                  predictedStateCovariance,
                                  predictedDistortedFeatures,
                                  predictedFeatureJacobians );
}

