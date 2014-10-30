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
#include "CommonFunctions.h"

#include "AddMapFeature.h"

// ------------------------------------------------------------------------------------------------------------

void undistortPoint(const double* imagePoint, double* imageUndistortedPoint)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;
    
    double pointMinusCenterX = imagePoint[0] - cameraCalib->cx;
    double pointMinusCenterY = imagePoint[1] - cameraCalib->cy;
    
    double dx = cameraCalib->dx * pointMinusCenterX;
    double dy = cameraCalib->dy * pointMinusCenterY;
    
    double rd_sq = dx * dx + dy * dy;
    
    double distortion = 1 + cameraCalib->k1 * rd_sq + cameraCalib->k2 * rd_sq * rd_sq;
    
    imageUndistortedPoint[0] = cameraCalib->cx + pointMinusCenterX * distortion;
    imageUndistortedPoint[1] = cameraCalib->cy + pointMinusCenterY * distortion;
}

// ------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la funcion que quita la distorsion a un punto
// Recibe featurePixels: 2x1
// Devuelve jacobian: 2x2

void computeUndistortPointJacobian(const double *featurePixels, double *jacobian)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    double ud = featurePixels[0];
    double vd = featurePixels[1];
    
    double xd = (ud - cameraCalib->cx) * cameraCalib->dx;
    double yd = (vd - cameraCalib->cy) * cameraCalib->dy;
    
    double rd2 = xd * xd + yd * yd;
    
    double k1_plus_2k2_rd2 = cameraCalib->k1 + 2.0L * cameraCalib->k2 * rd2;
    double k1_rd2_plus_k2_rd4_plus_1 = 1.0L + cameraCalib->k1 * rd2 + cameraCalib->k2 * rd2 * rd2;
    
    double _2dx2 = 2.0L * cameraCalib->dx * cameraCalib->dx;
    double _2dy2 = 2.0L * cameraCalib->dy * cameraCalib->dy;
    
    jacobian[0] = k1_rd2_plus_k2_rd4_plus_1 +
    (ud - cameraCalib->cx) * k1_plus_2k2_rd2 * ((ud - cameraCalib->cx) * _2dx2);
    jacobian[1] = (ud - cameraCalib->cx) * k1_plus_2k2_rd2 * ((vd - cameraCalib->cy) * _2dy2);
    jacobian[2] = (vd - cameraCalib->cy) * k1_plus_2k2_rd2 * ((ud - cameraCalib->cx) * _2dx2);
    jacobian[3] = (vd - cameraCalib->cy) * k1_plus_2k2_rd2 * ((vd - cameraCalib->cy) * _2dy2) +
    k1_rd2_plus_k2_rd4_plus_1;
    
}

// ------------------------------------------------------------------------------------------------------------
// Calcula el jacobiano de la funcion para inicializar un feature.
// Parametros:
// orientation = quaternion de orientacion
// rotationMatrixWC = quaternion pasado a matriz de rotacion
// featureImagePos = posicion del feature en la imgen (en pixels)
//
// Devuelve:
// jacobianPositionAndOrientation = matriz 6x7 del jacobiano que se corresponde con la derivada de la
//                                  funcion que agrega un feature a la covarianza con respecto a la posicion
//                                  de la camara y la orientacion
//
// jacobianHAndRho = matriz 6x3 del jacobiano que se corresponde con la derivada de la
//                                  funcion que agrega un feature a la covarianza con respecto a h y rho
//


void computeAddFeatureJacobian(const double *orientation,
                               const double *rotationMatrixWC,
                               const double *featureImagePos,
                               double *jacobianPositionAndOrientationSubmatrix,
                               double *jacobianHAndRhoSubmatrix)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;

    double undistortedPoint[2] = {0};
    undistortPoint(featureImagePos, undistortedPoint);
    
    // Feature con respecto a la camara pasado al espacio proyectivo
    double x_c = -(cameraCalib->cx - undistortedPoint[0]) / cameraCalib->fx;
    double y_c = -(cameraCalib->cy - undistortedPoint[1]) / cameraCalib->fy;
    double z_c = 1.0L;
    
    double xyz_c[3] = {0};
    xyz_c[0] = x_c;
    xyz_c[1] = y_c;
    xyz_c[2] = z_c;
    
    double xyz_w[3] = {0};
    multiplyRotationMatrixByVector(rotationMatrixWC, xyz_c, xyz_w);
    
    double x_w = xyz_w[0];
    double y_w = xyz_w[1];
    double z_w = xyz_w[2];
    
    double xx_plus_zz = x_w*x_w + z_w*z_w;
    
    double dtheta_dgw[3] = {0};
    dtheta_dgw[0] = z_w / xx_plus_zz;
    dtheta_dgw[1] = 0;
    dtheta_dgw[2] = -x_w / xx_plus_zz;
    
    double sqrt_xx_plus_zz = sqrt(x_w*x_w + z_w*z_w);
    double norm_sq = xx_plus_zz + y_w*y_w;
    
    double dphi_dgw[3] = {0};
    dphi_dgw[0] = x_w * y_w / (norm_sq * sqrt_xx_plus_zz);
    dphi_dgw[1] = -sqrt_xx_plus_zz / norm_sq;
    dphi_dgw[2] = z_w * y_w / (norm_sq * sqrt_xx_plus_zz);
    
    double dgw_dqwr[12] = {0};
    makeJacobianOfQuaternionToRotationMatrix(orientation, xyz_c, dgw_dqwr);
    
    double dtheta_dqwr[4] = {0};
    matrixMult(dtheta_dgw, 1, 3, dgw_dqwr, 4, dtheta_dqwr);
    
    double dphi_dqwr[4] = {0};
    matrixMult(dphi_dgw, 1, 3, dgw_dqwr, 4, dphi_dqwr);
    
    //    int derivativeResultRows = 6;
    int derivativeResultCols = 7;
    
    for (int i = 0; i < 3; ++i)
    {
        jacobianPositionAndOrientationSubmatrix[i*derivativeResultCols + i] = 1.0L;
    }
    
    for (int i = 0; i < 4; ++i)
    {
        // 4ta fila, columnas 3, 4, 5 y 6
        jacobianPositionAndOrientationSubmatrix[3*derivativeResultCols + i+3] = dtheta_dqwr[i];
        
        // 5ta fila, columnas 3, 4, 5 y 6
        jacobianPositionAndOrientationSubmatrix[4*derivativeResultCols + i+3] = dphi_dqwr[i];
    }
    
    // Para las siguientes lineas de codigo conviene ver el archivo de MATLAB add_a_feature_covariance_inverse_depth.m
    // Multiplicacion de dyprima_dgw * dgw_dgc.
    // subresult_dgw_dgc: 2x3
    double subresult_dgw_dgc[6] = {0};
    matrixMult(dtheta_dgw, 1, 3, rotationMatrixWC, 3, &subresult_dgw_dgc[0]);
    matrixMult(dphi_dgw, 1, 3, rotationMatrixWC, 3, &subresult_dgw_dgc[3]);
    
    // multiplicacion de dyprima_dgw * dgw_dgc * dgc_dhu.
    double fku_inv = 1.0L / cameraCalib->fx;
    double fkv_inv = 1.0L / cameraCalib->fy;
    
    // dgc_dhu: 3x2
    double dgc_dhu[6] = {fku_inv, 0, 0, fkv_inv, 0, 0};
    // subresult_dgc_dhu: 2x2
    double subresult_dgc_dhu[4] = {0};
    matrixMult(subresult_dgw_dgc, 2, 3, dgc_dhu, 2, subresult_dgc_dhu);
    
    double dhu_dhd[4] = {0};
    computeUndistortPointJacobian(featureImagePos, dhu_dhd);
    
    // jacobianHAndRhoSubmatrix:6x3 de la forma
    // 0   0   0
    // 0   0   0
    // 0   0   0
    // a   b   0
    // c   d   0
    // 0   0   1
    
    // subresult_dhu_dhd: 2x2
    double subresult_dhu_dhd[4] = {0};
    
    matrixMult(subresult_dgc_dhu, 2, 2, dhu_dhd, 2, subresult_dhu_dhd);
    
    jacobianHAndRhoSubmatrix[9] = subresult_dhu_dhd[0];
    jacobianHAndRhoSubmatrix[10] = subresult_dhu_dhd[1];
    jacobianHAndRhoSubmatrix[12] = subresult_dhu_dhd[2];
    jacobianHAndRhoSubmatrix[13] = subresult_dhu_dhd[3];
    jacobianHAndRhoSubmatrix[17] = 1.0L;
}

// ------------------------------------------------------------------------------------------------------------
// Agrega el feature a la matriz de covarianza

void addFeatureToCovarianceMatrix(const double *featureImagePos, State &state, Matd &stateCovarianceMatrix)
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;
    ExtendedKalmanFilterParameters *ekfParams = ConfigurationManager::getInstance().ekfParams;

    // jacobianPositionAndOrientation: 6x7 (ignoramos la parte de ceros que aparece en el paper Inverse Depth)
    // jacobianHAndRho: 6x3
    double jacobianPositionAndOrientation[42] = {0};
    double jacobianHAndRho[18] = {0};
    computeAddFeatureJacobian(state.orientation,
                              state.orientationRotationMatrix,
                              featureImagePos,
                              jacobianPositionAndOrientation,
                              jacobianHAndRho);
    
    // subMatrixCovarianceToAdd:3x3 es la matriz que se agrega abajo a la derecha en la covarianza:
    //
    // P = jacobiano * |covarianzaDelEstado             0            | * jacobiano'
    //                 |         0           subMatrixCovarianceToAdd|
    
    double subMatrixCovarianceToAdd[9] = {0};
    subMatrixCovarianceToAdd[0] = cameraCalib->pixelErrorX * cameraCalib->pixelErrorX;
    subMatrixCovarianceToAdd[4] = cameraCalib->pixelErrorY * cameraCalib->pixelErrorY;
    subMatrixCovarianceToAdd[8] = ekfParams->inverseDepthRhoSD * ekfParams->inverseDepthRhoSD;
    
    Matd covMatFirst7Rows(stateCovarianceMatrix, cv::Range(0, 7), cv::Range(0, stateCovarianceMatrix.cols));
    Matd covMatFirst7Cols(stateCovarianceMatrix, cv::Range(0, stateCovarianceMatrix.rows), cv::Range(0, 7));
    
    // Pasamos los jacobianos a Mat de OpenCV para poder realizar los calculos
    Matd jacobianPosAndOrientMat(6, 7, jacobianPositionAndOrientation);
    Matd jacobianHAndRhoMat(6, 3, jacobianHAndRho);
    
    // Declaramos la matriz que va a guardar los resultados
    Matd newStateCovarianceMatrix(stateCovarianceMatrix.rows + 6, stateCovarianceMatrix.cols + 6);
    
    // Submatriz inferior izquierda del resultado (las 6 filas que se agregaron, quitando las ultimas 6 columnas)
    Matd covMatNewFeatureVsPreviousState(newStateCovarianceMatrix,
                                         cv::Range(stateCovarianceMatrix.rows, newStateCovarianceMatrix.rows),
                                         cv::Range(0, stateCovarianceMatrix.cols));
    
    // Submatriz superior derecha del resultado (las 6 columnas que se agregaron, quitando las ultimas 6 filas)
    Matd covMatPreviousStateVsNewFeature(newStateCovarianceMatrix,
                                         cv::Range(0, stateCovarianceMatrix.rows),
                                         cv::Range(stateCovarianceMatrix.cols, newStateCovarianceMatrix.cols));
    
    // Submatriz inferior derecha del resultado (de 6x6, las ultimas 6 filas y 6 columnas)
    Matd covMatNewFeatureCovariance(newStateCovarianceMatrix,
                                    cv::Range(stateCovarianceMatrix.rows, newStateCovarianceMatrix.rows),
                                    cv::Range(stateCovarianceMatrix.cols, newStateCovarianceMatrix.cols));
    
    // Llenamos la matriz de resultado
    Matd(jacobianPosAndOrientMat * covMatFirst7Rows).copyTo(covMatNewFeatureVsPreviousState);
    Matd(covMatFirst7Cols * jacobianPosAndOrientMat.t()).copyTo(covMatPreviousStateVsNewFeature);
    
    Matd covMatNewFeatVsPrevStateFirst7Cols(covMatNewFeatureVsPreviousState,
                                            cv::Range(0, covMatNewFeatureVsPreviousState.rows),
                                            cv::Range(0, 7));
    
    Matd matFeatureNoise(3, 3, subMatrixCovarianceToAdd);
    Matd( covMatNewFeatVsPrevStateFirst7Cols * jacobianPosAndOrientMat.t() +
         jacobianHAndRhoMat * matFeatureNoise * jacobianHAndRhoMat.t() ).copyTo(covMatNewFeatureCovariance);
    
    stateCovarianceMatrix.copyTo( Matd( newStateCovarianceMatrix,
                                       cv::Range(0, stateCovarianceMatrix.rows),
                                       cv::Range(0, stateCovarianceMatrix.cols)) );
    
    // Devolvemos la matriz de resultado
    stateCovarianceMatrix = newStateCovarianceMatrix;
}

// ------------------------------------------------------------------------------------------------------------

void addFeatureToStateAndCovariance( const ImageFeatureMeasurement *imageFeatureMeasurement,
                                     State &state,
                                     Matd &stateCovarianceMatrix )
{
    CameraCalibration *cameraCalib = ConfigurationManager::getInstance().cameraCalibration;
    ExtendedKalmanFilterParameters *ekfParams = ConfigurationManager::getInstance().ekfParams;

    double featurePos[6] = {0.0L};
    
    // x y z
    for (int i = 0; i < 3; ++i)
    {
        featurePos[i] = state.position[i];
    }
    
    double undistortedImagePoint[2] = {0.0L};
    undistortPoint(imageFeatureMeasurement->imagePos, undistortedImagePoint);
    
    // Se retroproyecta el punto dedistorsionado
    double retroProjectedPoint[3] = {0.0L};
    retroProjectedPoint[0] = -(cameraCalib->cx - undistortedImagePoint[0]) / cameraCalib->fx;
    retroProjectedPoint[1] = -(cameraCalib->cy - undistortedImagePoint[1]) / cameraCalib->fy;
    retroProjectedPoint[2] = 1.0L;
    
    // Pasamos al eje de coordenadas del mundo
    multiplyRotationMatrixByVector(state.orientationRotationMatrix, retroProjectedPoint, retroProjectedPoint);
    
    // Extraemos la informacion
    double fdx = retroProjectedPoint[0];
    double fdy = retroProjectedPoint[1];
    double fdz = retroProjectedPoint[2];
    
    // theta
    featurePos[3] = atan2(fdx, fdz);
    
    // phi
    featurePos[4] = atan2(-fdy, sqrt(fdx*fdx + fdz*fdz));
    
    // rho
    featurePos[5] = ekfParams->initInvDepthRho;
    
    // Agregamos el feature al estado
    MapFeature *newMapFeature = new MapFeature( featurePos,
                                                6,
                                                stateCovarianceMatrix.cols,
                                                imageFeatureMeasurement->descriptorData,
                                                MAPFEATURE_TYPE_INVERSE_DEPTH );
    
    state.addFeature(newMapFeature);
    
    // Agregamos el MapFeaturea la matriz de covarianza
    addFeatureToCovarianceMatrix(imageFeatureMeasurement->imagePos, state, stateCovarianceMatrix);
    
#ifdef DEBUG_SHOW_IMAGES
    newMapFeature->lastImagePos[0] = imageFeatureMeasurement->imagePos[0];
    newMapFeature->lastImagePos[1] = imageFeatureMeasurement->imagePos[1];
#endif
}

// ------------------------------------------------------------------------------------------------------------

void addFeaturesToStateAndCovariance( const VectorImageFeatureMeasurement &imageFeatureMeasurement,
                                     State &state,
                                     Matd &stateCovarianceMatrix )
{
    size_t imageFeatureMeasurementSize = imageFeatureMeasurement.size();
    for (uint i = 0; i < imageFeatureMeasurementSize; ++i)
    {
        addFeatureToStateAndCovariance(imageFeatureMeasurement[i], state, stateCovarianceMatrix);
    }

#ifdef DEBUG
    std::cout << "Se agregaron " << imageFeatureMeasurementSize << " nuevos features al mapa." << std::endl;
#endif
}