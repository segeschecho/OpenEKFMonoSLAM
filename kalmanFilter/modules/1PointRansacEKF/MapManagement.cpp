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

#include "../Configuration/ConfigurationManager.h"
#include "../Core/EKFMath.h"

#include "CommonFunctions.h"

#include "MapFeature.h"
#include "State.h"

#include "DetectNewImageFeatures.h"

#include "MapManagement.h"

#if defined(DEBUG_SHOW_IMAGES)

// ------------------------------------------------------------------------------------------------------------

void cleanFeaturesLastImagePos(VectorMapFeature &features)
{
    size_t featuresSize = features.size();
    
    for (int i = 0; i < featuresSize; ++i)
    {
        features[i]->lastImagePos[0] = -1.0L;
        features[i]->lastImagePos[1] = -1.0L;
    }
}

// ------------------------------------------------------------------------------------------------------------

void updateFeaturesLastImagePos(VectorMapFeature &predictedFeatures, const VectorFeatureMatch &matches)
{
    size_t matchesSize = matches.size();
    
    for (int i = 0; i < matchesSize; ++i)
    {
        predictedFeatures[matches[i]->featureIndex]->lastImagePos[0] = matches[i]->imagePos[0];
        predictedFeatures[matches[i]->featureIndex]->lastImagePos[1] = matches[i]->imagePos[1];
    }
}

// ------------------------------------------------------------------------------------------------------------
// Funcion que actualiza los features del mapa: la cantidad de veces predicho, cantidad de veces matcheado, y descriptores
void updateMapFeatures(VectorImageFeaturePrediction &predictedDistortedFeatures, const VectorFeatureMatch& inlierMatches, State &state)
#else

void updateMapFeatures(const VectorImageFeaturePrediction &predictedDistortedFeatures, const VectorFeatureMatch& inlierMatches, State &state)
#endif
{
    size_t inlierMatchesSize = inlierMatches.size();
    size_t predictedDistortedFeaturesSize = predictedDistortedFeatures.size();
    
    for (int i = 0; i < predictedDistortedFeaturesSize; ++i)
    {
        int featureIndex = predictedDistortedFeatures[i]->featureIndex;
        MapFeature *currMapFeature = state.mapFeatures[featureIndex];
        currMapFeature->timesPredicted++;
    }
    
    for (int i = 0; i < inlierMatchesSize; ++i)
    {
        int featureIndex = inlierMatches[i]->featureIndex;
        MapFeature *currMapFeature = state.mapFeatures[featureIndex];
        currMapFeature->timesMatched++;
    }
#if defined(DEBUG_SHOW_IMAGES)
    // Asigno los valores correctos para la posicion en
    // la imagen de la ultima vez que se vieron los features
    cleanFeaturesLastImagePos(state.mapFeatures);
    updateFeaturesLastImagePos(state.mapFeatures, inlierMatches);
#endif
    
    // Actualizamos los descriptores de los inliers
    for (int i = 0; i < inlierMatchesSize; i++)
    {
        FeatureMatch *currFeatureMatch = inlierMatches[i];
        MapFeature *currMapFeature = state.mapFeatures[currFeatureMatch->featureIndex];
        
        assert(currMapFeature->descriptor.cols == currFeatureMatch->imagePosDescriptor.cols);
        
        currFeatureMatch->imagePosDescriptor.copyTo(currMapFeature->descriptor);
    }    
}

// ------------------------------------------------------------------------------------------------------------
// Importante: Los rangos deben estar ordenados y no deben solaparse
// Devuelve: la suma de los largos de los rangos resultado
int rangeComplement(const std::vector< cv::Range > ranges, int minValueRange, int maxValueRange, std::vector< cv::Range >& complement)
{
    complement.clear();
    
    size_t rangesSize = ranges.size();

    if (rangesSize == 0)
    {
        complement.push_back(cv::Range(minValueRange, maxValueRange));
        return maxValueRange - minValueRange;
    }

    int rangesSizeSum = 0;
    
    int lastEnd = ranges[0].end;
    if (ranges[0].start > minValueRange)
    {
        cv::Range complementSubRange(minValueRange, ranges[0].start);
        
        complement.push_back( complementSubRange );
        rangesSizeSum += complementSubRange.size();
    }

    for (uint i = 1; i < rangesSize; ++i)
    {
        const cv::Range &currRange = ranges[i];
        if (lastEnd < currRange.start)
        {
            cv::Range complementSubRange(lastEnd, currRange.start);

            complement.push_back( complementSubRange );
            rangesSizeSum += complementSubRange.size();
        }
        
        lastEnd = currRange.end;
    }
    
    if (lastEnd < maxValueRange)
    {
        cv::Range complementSubRange(lastEnd, maxValueRange);
        
        complement.push_back( complementSubRange );
        rangesSizeSum += complementSubRange.size();
    }
    
    return rangesSizeSum;
}

// ------------------------------------------------------------------------------------------------------------
// Importante: Los rangos rowsToRemove y columnsToRemove deben estar ordenados y no deben solaparse
Matd removeRowsAndColumnsFromMat(Matd matrix, const std::vector< cv::Range > rowsToRemove, const std::vector< cv::Range > columnsToRemove)
{
    std::vector< cv::Range > rows;
    std::vector< cv::Range > columns;
    
    int resultRows = rangeComplement(rowsToRemove, 0, matrix.rows, rows);
    int resultColumns = rangeComplement(columnsToRemove, 0, matrix.cols, columns);
    
    if (resultRows == 0 || resultColumns == 0)
    {
        return Matd();
    }
    
    size_t rowsSize = rows.size();
    size_t columnsSize = columns.size();

    Matd result( Matd::zeros(resultRows, resultColumns) );
    
    int nextEmptyRow = 0;
    for (uint i = 0; i < rowsSize; ++i)
    {
        int nextEmptyColumn = 0;

        const cv::Range& rowsRange = rows[i];
        const cv::Range subResultRowsRange(nextEmptyRow, nextEmptyRow + rowsRange.size());

        for (uint j = 0; j < columnsSize; ++j)
        {
            const cv::Range& columnsRange = columns[j];
            const cv::Range subResultColumnsRange(nextEmptyColumn, nextEmptyColumn + columnsRange.size());

            matrix(rowsRange, columnsRange).copyTo( result(subResultRowsRange, subResultColumnsRange) );

            nextEmptyColumn += columnsRange.size();
        }
        
        nextEmptyRow += rowsRange.size();
    }
    
    return result;
}

// ------------------------------------------------------------------------------------------------------------

void removeFeaturesFromStateAndCovariance(const VectorMapFeature &mapFeaturesToRemove, State &state, Matd &stateCovarianceMatrix)
{
    size_t mapFeaturesToRemoveSize = mapFeaturesToRemove.size();
    
    if (mapFeaturesToRemoveSize == 0)
    {
        return;
    }
    
    // Matriz de covarianza
    std::vector< cv::Range > rangesToRemove;
    for (uint i = 0; i < mapFeaturesToRemoveSize; ++i)
    {
        MapFeature *currMapFeature = mapFeaturesToRemove[i];
        cv::Range rangeToRemove( currMapFeature->covarianceMatrixPos,
                                 currMapFeature->covarianceMatrixPos + currMapFeature->positionDimension );
        
        assert( rangeToRemove.start >= 13 && rangeToRemove.end <= stateCovarianceMatrix.cols );

        rangesToRemove.push_back(rangeToRemove);
    }
    
    stateCovarianceMatrix = removeRowsAndColumnsFromMat(stateCovarianceMatrix, rangesToRemove, rangesToRemove);
    
    // State
    
    // Cambio la posicion en la matriz de la covarianza de todos aquellos features que
    // estan despues en la misma matriz que el feature a remover
    size_t mapFeaturesSize = state.mapFeatures.size();
    uint j = 0;
    uint removedAcum = 0;
    
    for (int i = 0; i < mapFeaturesSize; ++i)
    {
        MapFeature *currMapFeature = state.mapFeatures[i];
        if (j < mapFeaturesToRemoveSize && currMapFeature == mapFeaturesToRemove[j])
        {
            removedAcum += currMapFeature->positionDimension;
            j++;
        }
        else
        {
            currMapFeature->covarianceMatrixPos -= removedAcum;
        }
    }

    state.removeFeatures(mapFeaturesToRemove);
}

// ------------------------------------------------------------------------------------------------------------

void resetEKFMap(State &state, Matd &stateCovarianceMatrix)
{
    state = State(state.position, state.orientation, state.linearVelocity, state.angularVelocity);
    //    state.linearVelocity[0] = 0.0L;
    //    state.linearVelocity[1] = 0.0L;
    //    state.linearVelocity[2] = 0.0L;
    //
    //    state.angularVelocity[0] = EPSILON;
    //    state.angularVelocity[1] = EPSILON;
    //    state.angularVelocity[2] = EPSILON;
    
    initCovariance(stateCovarianceMatrix);
}

// ------------------------------------------------------------------------------------------------------------

void removeBadMapFeatures(State &state, Matd &stateCovarianceMatrix)
{
    // FIXME: La eliminacion se puede realizar de manera mas inteligente.
    // Para eliminar los features del mapFeatures se puede realizar una copia
    // del vector con los features buenos solamente, y luego asignar esa copia
    // al vector del estado.
    size_t mapFeaturesSize = state.mapFeatures.size();
    
    VectorMapFeature mapFeaturesToRemove;
    
    for (int i = 0; i < mapFeaturesSize; ++i)
    {
        MapFeature *currMapFeature = state.mapFeatures[i];
        
        // Si la cantidad de veces matcheado (o sea, cantidad de veces que es INLIER)
        // es menor a un cierto porcentaje de la cantidad de veces predicho,
        // eliminamos el feature ya que consideramos seria perjudicial para estimar
        float currMatchingPercent = static_cast<float>(currMapFeature->timesMatched) / static_cast<float>(currMapFeature->timesPredicted);
        if (currMatchingPercent < ConfigurationManager::getInstance().ekfParams->goodFeatureMatchingPercent)
        {
#ifdef DEBUG
            std::cout << "Se eliminará el feature de featureIndex " << i << std::endl;
#endif
            mapFeaturesToRemove.push_back(currMapFeature);
        }
    }
    
    removeFeaturesFromStateAndCovariance(mapFeaturesToRemove, state, stateCovarianceMatrix);
}

// ------------------------------------------------------------------------------------------------------------

double computeLinearityIndex(const State &state, const Matd &stateCovarianceMatrix, const MapFeature *mapFeature)
{
    int invDepthErrorCovMatrixIndex = mapFeature->covarianceMatrixPos + mapFeature->positionDimension - 1;
    
    double invDepthError = sqrt(stateCovarianceMatrix[invDepthErrorCovMatrixIndex][invDepthErrorCovMatrixIndex]);
    double invDepthValue = mapFeature->position[mapFeature->positionDimension - 1];
    
    double sigmaMapFeaturePosError = invDepthError / (invDepthValue * invDepthValue);
    
    double currMapFeatureXYZPos[3] = {0};
    changeInverseDepthToDepth(mapFeature->position, currMapFeatureXYZPos);
    
    double currMapFeatureXYZToCamPos[3] = {0.0L};
    double currMapFeatureXYZToFirstSeenCamPos[3] = {0.0L};
    
    matrixSubs(currMapFeatureXYZPos, state.position, 1, 3, currMapFeatureXYZToCamPos);
    matrixSubs(currMapFeatureXYZPos, mapFeature->position, 1, 3, currMapFeatureXYZToFirstSeenCamPos);
    
    double dotProduct = 0.0L;
    
    matrixMult(currMapFeatureXYZToCamPos, 1, 3, currMapFeatureXYZToFirstSeenCamPos, 1, &dotProduct);
    
    double currMapFeatureXYZToFirstSeenCamPosDistance = euclideanNorm3(currMapFeatureXYZToFirstSeenCamPos);
    double currMapFeatureXYZToCamPosDistance = euclideanNorm3(currMapFeatureXYZToCamPos);
    double cosAlphaDivDistance = dotProduct / (currMapFeatureXYZToFirstSeenCamPosDistance * currMapFeatureXYZToCamPosDistance);
    
    double result = 4.0L * sigmaMapFeaturePosError * cosAlphaDivDistance / currMapFeatureXYZToCamPosDistance;
    return result;
}

// ------------------------------------------------------------------------------------------------------------
// Funcion que pasa un feature del mapa de inverse depth a depth
void convertToDepth(MapFeature *mapFeature, State &state, Matd &stateCovarianceMatrix)
{
    double theta = mapFeature->position[3];
    double phi = mapFeature->position[4];
    double rho = mapFeature->position[5];

    double mi[3] = {0};
    double currMapFeatureXYZPos[3] = {0};
    
    makeDirectionalVector(theta, phi, mi);
    
    currMapFeatureXYZPos[0] = mapFeature->position[0] + mi[0]/rho;
    currMapFeatureXYZPos[1] = mapFeature->position[1] + mi[1]/rho;
    currMapFeatureXYZPos[2] = mapFeature->position[2] + mi[2]/rho;
    
    // Hacer cambios en la matriz de covarianza
    double dm_dtheta_div_rho[3] = {0};
    double dm_dphi_div_rho[3] = {0};
    double neg_mi_div_rho2[3] = {0};
    
    dm_dtheta_div_rho[0] = cos(phi) * cos(theta) / rho;
    dm_dtheta_div_rho[2] = -cos(phi) * sin(theta) / rho;
    
    dm_dphi_div_rho[0] = -sin(phi)*sin(theta) / rho;
    dm_dphi_div_rho[1] = -cos(phi) / rho;
    dm_dphi_div_rho[2] = -sin(phi)*cos(theta) / rho;

    neg_mi_div_rho2[0] = -mi[0] / (rho * rho);
    neg_mi_div_rho2[1] = -mi[1] / (rho * rho);
    neg_mi_div_rho2[2] = -mi[2] / (rho * rho);

    //
    // jacobian: 3x6
    //
    Matd jacobian( Matd::zeros(3, 6) );
    
    jacobian[0][0] = 1.0L;
    jacobian[1][1] = 1.0L;
    jacobian[2][2] = 1.0L;
    
    Matd(3, 1, dm_dtheta_div_rho).copyTo( jacobian(cv::Range(0, 3), cv::Range(3, 4)) );
    Matd(3, 1, dm_dphi_div_rho  ).copyTo( jacobian(cv::Range(0, 3), cv::Range(4, 5)) );
    Matd(3, 1, neg_mi_div_rho2  ).copyTo( jacobian(cv::Range(0, 3), cv::Range(5, 6)) );
    
    //
    // stateCovarianceMatrix =  | P_{k x k}         P_{k x 6}      P_{k x n-k-6}     |
    //                          | P_{6 x k}         P_{6 x 6}      P_{6 x n-k-6}     | <- P_{6 x n}
    //                          | P_{n-k-6 x k}     P_{n-k-6 x 6}  P_{n-k-6 x n-k-6} |
    //
    
    int covarianceMatrixPos = mapFeature->covarianceMatrixPos;
    int oldPosDimension = mapFeature->positionDimension;
    
    cv::Range firstKRange(0, covarianceMatrixPos);
    cv::Range featureRange(covarianceMatrixPos, covarianceMatrixPos + oldPosDimension);
    cv::Range lastRange(covarianceMatrixPos + oldPosDimension, stateCovarianceMatrix.cols);

    cv::Range newCovFeatureRange(covarianceMatrixPos, covarianceMatrixPos + 3);
    cv::Range newCovMatLastRange(covarianceMatrixPos + 3, stateCovarianceMatrix.cols - 3);

    bool lastFeatureInCovMatrix = lastRange.start == lastRange.end || newCovMatLastRange.start == newCovMatLastRange.end;

    Matd stateCovMatrix_6xn( stateCovarianceMatrix, featureRange, cv::Range(0, stateCovarianceMatrix.cols) );
    Matd stateCovMatrix_kx6( stateCovarianceMatrix, firstKRange, featureRange );
    Matd stateCovMatrix_nk6x6( stateCovarianceMatrix, lastRange, featureRange );

    Matd stateCovMatrix_kxk( stateCovarianceMatrix, firstKRange, firstKRange );
    Matd stateCovMatrix_nk6xk( stateCovarianceMatrix, lastRange, firstKRange );
    Matd stateCovMatrix_kxnk6( stateCovarianceMatrix, firstKRange, lastRange );
    Matd stateCovMatrix_nk6xnk6( stateCovarianceMatrix, lastRange, lastRange );

    Matd newCovarianceMatrix( Matd::zeros(stateCovarianceMatrix.rows - 3, stateCovarianceMatrix.cols - 3) );

    Matd subResult_3xn(jacobian * stateCovMatrix_6xn);
    Matd subResult_3xk( subResult_3xn, cv::Range(0, subResult_3xn.rows), firstKRange );
    Matd subResult_3x6( subResult_3xn, cv::Range(0, subResult_3xn.rows), featureRange );
    
    Matd(subResult_3x6 * jacobian.t()).copyTo( newCovarianceMatrix(newCovFeatureRange, newCovFeatureRange) );

    subResult_3xk.copyTo( newCovarianceMatrix(newCovFeatureRange, firstKRange) );
    if (!lastFeatureInCovMatrix)
    {
        Matd(subResult_3xn, cv::Range(0, subResult_3xn.rows), lastRange).copyTo( newCovarianceMatrix(newCovFeatureRange, newCovMatLastRange) );
    }
    
    Matd(stateCovMatrix_kx6 * jacobian.t()).copyTo( newCovarianceMatrix(firstKRange, newCovFeatureRange) );
    if (!lastFeatureInCovMatrix)
    {
        Matd(stateCovMatrix_nk6x6 * jacobian.t()).copyTo( newCovarianceMatrix(newCovMatLastRange, newCovFeatureRange) );
    }

    stateCovMatrix_kxk.copyTo( newCovarianceMatrix(firstKRange, firstKRange) );
    
    if (!lastFeatureInCovMatrix)
    {
        stateCovMatrix_nk6xk.copyTo( newCovarianceMatrix(newCovMatLastRange, firstKRange) );
        stateCovMatrix_kxnk6.copyTo( newCovarianceMatrix(firstKRange, newCovMatLastRange) );
        stateCovMatrix_nk6xnk6.copyTo( newCovarianceMatrix(newCovMatLastRange, newCovMatLastRange) );
    }

    stateCovarianceMatrix = newCovarianceMatrix;
    
    // Hacer cambios en el state
    mapFeature->positionDimension = 3; // XYZ parameters
    
    delete [] mapFeature->position;
    mapFeature->position = new double[mapFeature->positionDimension];
    mapFeature->position[0] = currMapFeatureXYZPos[0];
    mapFeature->position[1] = currMapFeatureXYZPos[1];
    mapFeature->position[2] = currMapFeatureXYZPos[2];
    
    mapFeature->featureType = MAPFEATURE_TYPE_DEPTH;
    
    state.mapFeaturesDepth.push_back(mapFeature);
    
    VectorMapFeature::iterator itInvDepthMapFeature = state.mapFeaturesInvDepth.begin();
    VectorMapFeature::iterator itInvDepthMapFeatureEnd = state.mapFeaturesInvDepth.end();
    
    while(itInvDepthMapFeature != itInvDepthMapFeatureEnd && mapFeature != *itInvDepthMapFeature)
    {
        itInvDepthMapFeature++;
    }
    
    assert(itInvDepthMapFeature != itInvDepthMapFeatureEnd);

    state.mapFeaturesInvDepth.erase(itInvDepthMapFeature);

    VectorMapFeature::iterator itMapFeature = state.mapFeatures.begin();
    VectorMapFeature::iterator itMapFeatureEnd = state.mapFeatures.end();
    
    while(itMapFeature != itMapFeatureEnd && mapFeature != *itMapFeature)
    {
        itMapFeature++;
    }
    
    assert(itMapFeature != itMapFeatureEnd);
    
    size_t mapFeaturesSize = state.mapFeatures.size();
    int newPosDimension = mapFeature->positionDimension;
    for(uint i = 0; i < mapFeaturesSize; ++i)
    {
        int &currMapFeatureCovarianceMatPos = state.mapFeatures[i]->covarianceMatrixPos;
        if (currMapFeatureCovarianceMatPos > covarianceMatrixPos)
        {
            currMapFeatureCovarianceMatPos = currMapFeatureCovarianceMatPos - oldPosDimension + newPosDimension;
        }
    }
}

// ------------------------------------------------------------------------------------------------------------

void convertMapFeaturesInverseDepthToDepth(State &state, Matd &stateCovarianceMatrix)
{
    double linearityIndexThreshold = ConfigurationManager::getInstance().ekfParams->inverseDepthLinearityIndexThreshold;
    
    size_t mapFeaturesInvDepthSize = state.mapFeaturesInvDepth.size();
    
    for (uint i = 0; i < mapFeaturesInvDepthSize; ++i)
    {
        MapFeature *currMapFeature = state.mapFeaturesInvDepth[i];
        
        double linearityIndexValue = computeLinearityIndex(state, stateCovarianceMatrix, currMapFeature);
        
        // Si linearityIndexValue es menor al threshold, convertir a depth
        if (linearityIndexValue < linearityIndexThreshold)
        {
            convertToDepth(currMapFeature, state, stateCovarianceMatrix);
#ifdef DEBUG
            size_t mapFeaturesSize = state.mapFeatures.size();
            uint i = 0;
            while (i < mapFeaturesSize && state.mapFeatures[i] != currMapFeature)
            {
                ++i;
            }
            
            std::cout << "Se convirtió a DEPTH el feature de índice " << i << std::endl;
#endif
            return; // hacemos un return para no convertir todos en el mismo frame
        }
    }
}

// ------------------------------------------------------------------------------------------------------------