#!/bin/sh

#Copyright (C) 2013 Sergio E. Gonzalez and Emiliano D. González
#Facultad de Ciencias Exactas y Naturales, Universidad de Buenos Aires, Buenos Aires, Argentina.
# 
#C/C++, Java and XML/YML code for EKF SLAM from a monocular sequence.
#
#This file is part of OpenEKFMonoSLAM.
#
#OpenEKFMonoSLAM is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.
#
#OpenEKFMonoSLAM is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.
#
#You should have received a copy of the GNU General Public License
#along with OpenEKFMonoSLAM.  If not, see <http:www.gnu.org/licenses/>.
#
#If you use this code for academic work, please reference:
#GONZALEZ, S.; E. GONZÁLEZ; M. NITSCHE; P. DE CRISTÓFORIS. "Odometría Visual para Robots Móviles Utilizando Smartphones como Unidad de Sensado y Procesamiento". En: Jornadas Argentinas de Robótica, 8as : 2014 : Ciudad Autónoma de Buenos Aires. Actas : VIII Jornadas Argentinas de Robótica . (8 : Ciudad Autónoma de Buenos Aires, 12-14 de noviembre 2014).
#
#Authors:    Sergio E. Gonzalez - segonzalez@dc.uba.ar
#            Emiliano D. González - edgonzalez@dc.uba.ar
#
#Departamento de Computación
#Facultad de Ciencias Exactas y Naturales
#Universidad de Buenos, Buenos Aires, Argentina
#Date: June 2013

# Este archivo copia las fuentes de la version C++ del EKF dentro de las carpetas en jni
# para poder compilar la version Android

# $1 tiene que apuntar al directorio MODULES de las fuentes del EKF

echo Se van a copiar los modulos de: $1 al directorio actual
echo ""

# carpeta 1PointRansacEKF
echo Copiando 1PointRansacEKF
for i in $1/1PointRansacEKF/*.cpp $1/1PointRansacEKF/*.h; do
    cp $i EKF/modules/1PointRansacEKF/
done

# carpeta Core
echo Copiando Core
for i in $1/Core/*.cpp $1/Core/*.h; do
    cp $i EKF/modules/Core/
done

# carpeta Gui
echo Copiando Gui
for i in $1/Gui/*.cpp $1/Gui/*.h; do
    cp $i EKF/modules/Gui/
done

# carpeta Configuration
echo Copiando Configuration
for i in $1/Configuration/*.cpp $1/Configuration/*.h; do
    cp $i EKF/modules/Configuration/
done

echo Copiando Configuration/ConfigurationDataReader
# carpeta Configuration/ConfigurationDataReader
for i in $1/Configuration/ConfigurationDataReader/*.cpp $1/Configuration/ConfigurationDataReader/*.h; do
    cp $i EKF/modules/Configuration/ConfigurationDataReader/
done

# carpeta Configuration/ConfigurationDataReader/CameraCalibrationConfiguration
echo Copiando  Configuration/ConfigurationDataReader/CameraCalibrationConfiguration
for i in $1/Configuration/ConfigurationDataReader/CameraCalibrationConfiguration/*.cpp $1/Configuration/ConfigurationDataReader/CameraCalibrationConfiguration/*.h ; do
    cp $i EKF/modules/Configuration/ConfigurationDataReader/CameraCalibrationConfiguration/
done

# carpeta DescriptorExtractorConfiguration
for i in $1/Configuration/ConfigurationDataReader/DescriptorExtractorConfiguration/*.cpp $1/Configuration/ConfigurationDataReader/DescriptorExtractorConfiguration/*.h; do
    cp $i EKF/modules/Configuration/ConfigurationDataReader/DescriptorExtractorConfiguration/
done

# carpeta ExtendedKalmanFilterConfiguration
for i in $1/Configuration/ConfigurationDataReader/ExtendedKalmanFilterConfiguration/*.cpp $1/Configuration/ConfigurationDataReader/ExtendedKalmanFilterConfiguration/*.h; do
    cp $i EKF/modules/Configuration/ConfigurationDataReader/ExtendedKalmanFilterConfiguration/
done

# carpeta FeatureDetectorConfiguration
for i in $1/Configuration/ConfigurationDataReader/FeatureDetectorConfiguration/*.cpp $1/Configuration/ConfigurationDataReader/FeatureDetectorConfiguration/*.h; do
    cp $i EKF/modules/Configuration/ConfigurationDataReader/FeatureDetectorConfiguration/
done

echo Copiando ImageGenerator
# carpeta ImageGenerator
for i in $1/ImageGenerator/*.cpp $1/ImageGenerator/*.h; do
    cp $i EKF/modules/ImageGenerator/
done




