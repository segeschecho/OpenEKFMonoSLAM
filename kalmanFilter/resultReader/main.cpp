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

#include <opencv/cv.h>
#include "../modules/Core/Base.h"

#include <iostream>
#include "Points3d.h"
#include "Points1d.h"
#include "ScaleFactor.h"
#include "TimesCpu.h"
#include "../modules/Core/EKFMath.h"

#define EXABOT_VELOCITY 0.002904L   //velocidad lineal del exabot en Metros/seg manejandolo con un valor de motores 0.3

int main(int argc, const char* argv[])
{
    if (argc < 3)
    {
        std::cerr << "missing path to output.yml and path to the output files" << std::endl;
        return -1;
    }

    const char *fileName = argv[1];
    const std::string outputPath(argv[2]);

    cv::FileStorage fileStorage(fileName, cv::FileStorage::READ);

    if (!fileStorage.isOpened())
    {
        std::cerr << "File " << fileName << " could not be opened to load configuration." << std::endl;
        return false;
    }

    cv::FileNode root = fileStorage.root();
    cv::FileNodeIterator it_end = root.end();

    Points3d cameraPositions;
    Points3d cameraLinearVelocities;
    Points3d cameraAngularVelocities;
    Points3d cameraOrientation;
    ScaleFactor scaleFactor;

    std::vector<double> scaleFactors;
    double minFactor = 99999999.0L;
    double maxFactor = 0.0L;

    Points1d matches;
    Points1d inliers;
    Points1d rescued;

    TimesCpu times;

    for(cv::FileNodeIterator it = root.begin() ; it != it_end; ++it)
    {
        // Estado de la camara.
        Matd state;

        (*it)["StateEstimation"] >> state;

        cameraPositions.add(state[0][0], state[0][1], state[0][2]);
        cameraLinearVelocities.add(state[0][7], state[0][8], state[0][9]);
        cameraAngularVelocities.add(state[0][10], state[0][11], state[0][12]);

        // la orientacion se pasa a 3d en vez de dejarla en cuaternions
        double q[4] = {state[0][3], state[0][4], state[0][5], state[0][6]};
        double angles[3] = {0};

        quaterionToAngles(q, angles);
        cameraOrientation.add(angles[0], angles[1], angles[2]);

        // Se calcula el factor de escala en base a la longitud del vector dirección y la velocidad del exabot f = velReal / velEstimada
        double velocity_long = sqrt(state[0][7]*state[0][7] + state[0][8]*state[0][8] + state[0][9]*state[0][9]);

        if (velocity_long > 0)
        {
            double factor = EXABOT_VELOCITY / velocity_long;
            scaleFactors.push_back(factor);

            if (factor < minFactor)
            {
                minFactor = factor;
            }

            if (factor > maxFactor)
            {
                maxFactor = factor;
            }
        }

        // Se obtiene la informacion de los matches, inliers y rescatados
        int actualMatches = 0;
        int actualInliers = 0;
        int actualRescued = 0;

        (*it)["totalMatches"] >> actualMatches;
        (*it)["liInliers"] >> actualInliers;
        (*it)["hiInliers"] >> actualRescued;

        matches.add(actualMatches);
        inliers.add(actualInliers);
        rescued.add(actualRescued);

        // Se obtienen los tiempos de cpu de cada parte
        double prediction = 0.0L;
        double matching = 0.0L;
        double ransac = 0.0L;
        double li = 0.0L;
        double rescue = 0.0L;
        double hi = 0.0L;
        double map = 0.0L;

        (*it)["Prediction"] >> prediction;
        (*it)["Matching"] >> matching;
        (*it)["Ransac"] >> ransac;
        (*it)["UpdateLI"] >> li;
        (*it)["RescueOutliers"] >> rescue;
        (*it)["UpdateHI"] >> hi;
        (*it)["MapManagement"] >> map;

        times.add(prediction, matching, ransac, li, rescue, hi, map);
    }

    // Se determina el factor de escala del mapa en base a la velocidad del robot.
    scaleFactor.determine(scaleFactors, minFactor, maxFactor);
    // Una vez que se tienen los datos necesarios se guardan
    cameraPositions.save(outputPath, "cameraPositions");
    cameraLinearVelocities.save(outputPath, "cameraLinearVelocities");
    cameraAngularVelocities.save(outputPath, "cameraAngularVelocities");
    cameraOrientation.save(outputPath, "cameraOrientation");
    scaleFactor.save(outputPath, "scaleFactor");
    matches.save(outputPath,  "matches");
    inliers.save(outputPath, "inliers");
    rescued.save(outputPath, "rescued");
    times.save(outputPath, "timesCpu");

    fileStorage.release();

    return 0;
}
