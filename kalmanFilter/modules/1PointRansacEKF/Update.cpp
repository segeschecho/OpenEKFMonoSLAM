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
#include "ImageFeaturePrediction.h"

#include "Update.h"

// ------------------------------------------------------------------------------------------------------------
// Jacobiano de la funcion que normaliza un cuaternion:
// q = a + ix + jy + kz
// norm(q) = (a + ix + jy + kz)/sqrt(a*a + x*x + y*y + z*z)
// ------------------------------------------------------------------------------------------------------------
double normalizeQuaternionJacobian(const double *q, Matd &jacobian)
{
     double r = q[0];
     double x = q[1];
     double y = q[2];
     double z = q[3];
     double norm = sqrt(r*r + x*x + y*y + z*z);
     double a = 1.0L/(pow(norm, 3));

    jacobian = Matd((Matd(4,4) <<  x*x+y*y+z*z, -r*x,         -r*y,         -r*z,
                                  -x*r,          r*r+y*y+z*z, -x*y,         -x*z,
                                  -y*r,         -y*x,          r*r+x*x+z*z, -y*z,
                                  -z*r,         -z*x,         -z*y,          r*r+x*x+y*y)) * a;

    return norm;
}

// ------------------------------------------------------------------------------------------------------------

void normalizeCovariance(Matd &covariance, const Matd &normQuaternionJacobian)
{
    // Se va a hacer p_k_k(1:3,4:7)*Jnorm'
    Matd subMatrix(covariance, cv::Range(0, 3), cv::Range(3, 7));
    Matd(subMatrix * normQuaternionJacobian.t()).copyTo(subMatrix);

    // Se va a hacer Jnorm*p_k_k(4:7,1:3)
    subMatrix = Matd(covariance, cv::Range(3, 7), cv::Range(0, 3));
    Matd(normQuaternionJacobian * subMatrix).copyTo(subMatrix);

    // Se va a hacer Jnorm*p_k_k(4:7,4:7)*Jnorm'
    subMatrix = Matd(covariance, cv::Range(3, 7), cv::Range(3, 7));
    Matd(normQuaternionJacobian * subMatrix * normQuaternionJacobian.t()).copyTo(subMatrix);

    // Se va a hacer Jnorm*p_k_k(4:7,8:size_p_k_k)
    subMatrix = Matd(covariance, cv::Range(3, 7), cv::Range(7, covariance.cols));
    Matd(normQuaternionJacobian * subMatrix).copyTo(subMatrix);

    // Se va a hacer p_k_k(8:size_p_k_k,4:7)*Jnorm'
    subMatrix = Matd(covariance, cv::Range(7, covariance.rows), cv::Range(3, 7));
    Matd(subMatrix * normQuaternionJacobian.t()).copyTo(subMatrix);
}

// ------------------------------------------------------------------------------------------------------------
// calcula el kalman Gain
// K = P * H' * inv(S)
// Resultado: matriz (13 + 3d + 6id)x(2xfeaturesPredichos)
// ------------------------------------------------------------------------------------------------------------
void determineKalmanGain(const Matd &predictedFeatureJacobians, const Matd &predictedStateCovariance,
                         uint numberOfpredictedFeatures, Matd &kalmanGain)
{
    double cameraPixelError = ConfigurationManager::getInstance().cameraCalibration->pixelErrorX;

    Matd R( Matd::eye(2*numberOfpredictedFeatures, 2*numberOfpredictedFeatures) * cameraPixelError );
    Matd S;

    // predictedStateCovariance: NxN
    // predictedFeatureJacobians: (2xfeaturesPredichos)xn
    
    // S tiene dimension (2xfeaturesPredichos)x(2xfeaturesPredichos)
    // S = H*P*H' + R
    Matd stateCovMultPredJacob(predictedStateCovariance * predictedFeatureJacobians.t());
    
    Matd(predictedFeatureJacobians * stateCovMultPredJacob + R).copyTo(S);
    Matd(stateCovMultPredJacob * S.inv()).copyTo(kalmanGain);
}


// ------------------------------------------------------------------------------------------------------------
// Actualiza el estado, es decir el estado de la camara y la posicion de los features
// x = x + K(z - h)
// ------------------------------------------------------------------------------------------------------------
void stateUpdate(const Matd &kalmanGain, const VectorFeatureMatch &measurementMatchedFeatures,
            const VectorImageFeaturePrediction &predictedDistortedFeatures, State &state)
{
    size_t totalNumberOfFeatures = state.mapFeatures.size();
    size_t numberOfpredictedFeatures = predictedDistortedFeatures.size();

    // matriz para guardar la diferencia entre los puntos en la imagen medidos vs los predichos
    double *imagePointsDifference = new double[2*numberOfpredictedFeatures];

    for (uint i = 0; i < numberOfpredictedFeatures; ++i)
    {
        FeatureMatch *matchedFeature = measurementMatchedFeatures[i];
        ImageFeaturePrediction *predictedDistortedFeature = predictedDistortedFeatures[i];

        double imagePointsDifferenceX = matchedFeature->imagePos[0] - predictedDistortedFeature->imagePos[0];
        double imagePointsDifferenceY = matchedFeature->imagePos[1] - predictedDistortedFeature->imagePos[1];
        
        imagePointsDifference[2*i] = fabs(imagePointsDifferenceX) > DELTA ? imagePointsDifferenceX : 0.0L;
        imagePointsDifference[2*i + 1] = fabs(imagePointsDifferenceY) > DELTA ? imagePointsDifferenceY : 0.0L;;
    }

    Matd measurementDifference(2*static_cast<int>(numberOfpredictedFeatures), 1, imagePointsDifference);

    // kalmanGain: (13 + 3d + 6id)x2(d+id)
    // measurementDifference: 2(d+id)x1
    // kByDifference: (13 + 3d + 6id) x 1

    Matd kByDifference;
    Matd(kalmanGain * measurementDifference).copyTo(kByDifference);
    double *kByDPointer = kByDifference.ptr<double>();

    // ------------------------------------
    // se actualizan los datos de la camara
    double sumValue = 0;
    for (uint i = 0; i < 3; ++i)
    {
        sumValue = kByDPointer[i];
        if (fabs(sumValue) > DELTA)
        {
            state.position[i] += sumValue;
        }
    }

    for (uint i = 0; i < 4; ++i)
    {
        sumValue = kByDPointer[i + 3];
        if (fabs(sumValue) > DELTA)
        {
            state.orientation[i] += sumValue;
        }
    }

    state.setOrientation(state.orientation);

    for (uint i = 0; i < 3; ++i)
    {
        sumValue = kByDPointer[i + 7];
        if (fabs(sumValue) > DELTA)
        {
            state.linearVelocity[i] += sumValue;
        }
    }

    for (uint i = 0; i < 3; ++i)
    {
        sumValue = kByDPointer[i + 10];
        if (fabs(sumValue) > DELTA)
        {
            state.angularVelocity[i] += sumValue;
        }
    }

    // se recorren todos los features y se actualizan con la cuenta
    uint dimensionAcum = 13;
    for (uint i = 0; i < totalNumberOfFeatures; ++i)
    {
        int posDimension = state.mapFeatures[i]->positionDimension;

        for (uint j = 0; j < posDimension; ++j)
        {
            sumValue = kByDPointer[dimensionAcum];
            if (fabs(sumValue) > DELTA)
            {
                state.mapFeatures[i]->position[j] += sumValue;
            }
            
            dimensionAcum++;
        }
    }

    // se borran los vectores dinamicos.
    delete [] imagePointsDifference;
}

// ------------------------------------------------------------------------------------------------------------
// Actualiza la matriz de covarianza
// P = (I - K*H)*P
// ------------------------------------------------------------------------------------------------------------
void covarianceUpdate(const Matd &kalmanGain, const Matd &predictedFeatureJacobians, Matd &stateCovariance)
{
    Matd( (Matd::eye(stateCovariance.rows, stateCovariance.rows) - kalmanGain * predictedFeatureJacobians) *
          stateCovariance ).copyTo(stateCovariance);
}

// ------------------------------------------------------------------------------------------------------------

void joinJacobians(const VectorMatd &predictedFeatureJacobians, uint numberOfpredictedFeatures, Matd &jacobiansJoin)
{
    for (uint i = 0; i < numberOfpredictedFeatures; ++i)
    {
        // la matriz con el join de los jacobianos se crea con las referencias de lo que esta en predictedFeatureJacobians
        // esta matriz no se va a modificar, y esta asignación es O(1) (segun la referencia de opencv)
        // TODO: Ver si esta parte se puede optimizar, para que haga copias
        predictedFeatureJacobians[i]->row(0).copyTo(jacobiansJoin.row(2*i));
        predictedFeatureJacobians[i]->row(1).copyTo(jacobiansJoin.row(2*i + 1));
    }
}

// ------------------------------------------------------------------------------------------------------------
// Actualiza el estado y la covarianza, calculando el kalmanGain adentro.
// ------------------------------------------------------------------------------------------------------------
void updateStateAndCovariance(const VectorImageFeaturePrediction &predictedDistortedFeatures,
                              const VectorFeatureMatch &measurementMatchedFeatures,
                              const VectorMatd &predictedFeatureJacobians,
                              bool updateCovariance,
                              State &state, Matd &covariance)

{
    int numberOfpredictedFeatures = static_cast<int>(predictedDistortedFeatures.size());
    int stateDimension = 13 + 3*static_cast<int>(state.mapFeaturesDepth.size()) + 6*static_cast<int>(state.mapFeaturesInvDepth.size());

    // matriz que junta todos los jacobianos de los features
    Matd predictedFeatureJacobiansJoin(Matd::zeros(2*numberOfpredictedFeatures, stateDimension));
    joinJacobians(predictedFeatureJacobians, numberOfpredictedFeatures, predictedFeatureJacobiansJoin);

    Matd kalmanGain(Matd::zeros(stateDimension, 2*numberOfpredictedFeatures));
    determineKalmanGain(predictedFeatureJacobiansJoin, covariance, numberOfpredictedFeatures, kalmanGain);

    // -----------------------------------------
    // Actualizacion del estado

    stateUpdate(kalmanGain, measurementMatchedFeatures, predictedDistortedFeatures, state);

    // -------------------------------
    // Actualizacion de la covarianza
    if (updateCovariance)
    {
        covarianceUpdate(kalmanGain, predictedFeatureJacobiansJoin, covariance);
    }
}

// ------------------------------------------------------------------------------------------------------------
// actualiza solo el estado, no la covarianza
void updateOnlyState(const VectorImageFeaturePrediction &predictedDistortedFeatures,
                     const VectorFeatureMatch &measurementMatchedFeatures,
                     const VectorMatd &predictedFeatureJacobians,
                     State &state, Matd &covariance)
{
    updateStateAndCovariance(predictedDistortedFeatures, measurementMatchedFeatures, predictedFeatureJacobians, false, state, covariance);
}

// ------------------------------------------------------------------------------------------------------------
// Actualiza el estado y la matriz de la covarianza del estado en base a los puntos sensados y a la prediccion
// tanto del estado, su covarianza y los features dentro de la imagen.
// devuelve el estado actualizado junto con su matriz de covarianza.
// ------------------------------------------------------------------------------------------------------------
void update( State &state,
             Matd &covariance,
             const VectorFeatureMatch &measurementMatchedFeatures,
             const VectorImageFeaturePrediction &predictedDistortedFeatures,
             const VectorMatd &predictedFeatureJacobians )
{
    // -----------------------------------------------------------------
    // Calculo del Kalman Gain y actualizacion del estado y covarianza

    // Si se encontraron matches, entonces se hace el calculo del kalmanGain, Si no no hace falta.
    if (measurementMatchedFeatures.size() > 0)
    {
        updateStateAndCovariance(predictedDistortedFeatures, measurementMatchedFeatures, predictedFeatureJacobians, true, state, covariance);

        // -------------------------------------
        // Cuentas para evitar erroes numericos

        Matd normQuaternionJacobian;
        double quaternionNorm;

        // Se trata de dejar la matriz de covarianza los mas simetrica posible
        Matd(0.5L*covariance + 0.5L*covariance.t()).copyTo(covariance);

        quaternionNorm = normalizeQuaternionJacobian(state.orientation, normQuaternionJacobian);

        // Se normaliza el cuaternion
        double normalizedQuaternion[4] = {state.orientation[0] / quaternionNorm,
                                          state.orientation[1] / quaternionNorm,
                                          state.orientation[2] / quaternionNorm,
                                          state.orientation[3] / quaternionNorm};

        state.setOrientation(normalizedQuaternion);

        // Se modifica la matriz de covarianza con el quaternion normalizado

        normalizeCovariance(covariance, normQuaternionJacobian);
    }
}
