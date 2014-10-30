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

#include "ScaleFactor.h"
#include <iostream>
#include <iomanip>

ScaleFactor::ScaleFactor()
{
    factor = 0;
}

ScaleFactor::~ScaleFactor(){}

void ScaleFactor::determine(const std::vector<double> &scaleFactors, double minFactor, double maxFactor)
{
    // Se realiza un histograma de los factores de escala para determinar el que tiene mas valores y luego hacer su moda, para obtener el factor de escala final
    // 30 buckets con respecto a la longitud entre el factor mas chico y el mas grande.
    std::vector<int> histogram(HISTOGRAM_BUCKETS, 0);
    std::vector< std::vector<double> > voteFactors(HISTOGRAM_BUCKETS, std::vector<double>());
    double bucketSize = (maxFactor - minFactor)/HISTOGRAM_BUCKETS;

    for (unsigned int i = 0; i < scaleFactors.size(); ++i)
    {
        double f = scaleFactors[i];

        int pos = (int)((f - minFactor) / bucketSize);

        if (pos == HISTOGRAM_BUCKETS)
        {
            pos--;
        }

        voteFactors[pos].push_back(f);
        histogram[pos]++;
    }

    // se busca el bucket con mas votos.
    int maxVotes = 0;
    int maxBucketPos = 0;

    for (unsigned int i = 0; i < histogram.size(); ++i)
    {
        int votes = histogram[i];

        if (votes > maxVotes)
        {
            maxVotes = votes;
            maxBucketPos = i;
        }
    }

    // Se calcula el promedio para determinar el factor de escala final
    double sum = 0;
    for (unsigned int i = 0; i < voteFactors[maxBucketPos].size(); ++i)
    {
        sum += voteFactors[maxBucketPos][i];
    }

    factor = sum / voteFactors[maxBucketPos].size();
}

void ScaleFactor::save(std::string dirname, std::string filename)
{
    std::ofstream f;
    std::string file = dirname + filename + ".m";
    f.open(file.c_str());

    // Se guarda el factor

    f << "function factor = " << filename << "()" << std::endl;

    f << std::setprecision(12) << "    factor = " << factor << ";" << std::endl;

    f << "end";

    f.close();
}
