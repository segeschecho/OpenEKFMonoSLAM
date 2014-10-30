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

package tesis.ekfmonoslam;

import android.util.Log;


public class EKF {
    private long _ekfNativeReference = 0;
    private String _configFilePath;
    private String _outputPath;
    private String TAG = "EKF";
    
    private double xCam = 0.0;
    private double yCam = 0.0;
    private double zCam = 0.0;
    
    private int _cameraPreviewWidth;
    private int _cameraPreviewHeight;
    
    private boolean _isRunning = false;
    
    static {
    	System.loadLibrary("EKFNative");
    }

    private native void loadEKFNativeReference(String configurationFilePath, String outputPath);
    private native void EKFInit(byte[] imageData, int imageWidth, int imageHeight);
    private native void EKFStep(byte[] imageData, int imageWidth, int imageHeight);
    private native void releaseEKFNativeReference();
    private native void loadEKFCameraPreviewDimensions();
    
    public EKF(String configurationFilePath, String outputPath, int cameraPreviewWidth, int cameraPreviewHeight) {
        _configFilePath = configurationFilePath;
        _outputPath = outputPath;
        
        _cameraPreviewWidth = cameraPreviewWidth;
        _cameraPreviewHeight = cameraPreviewHeight;
    }
    
    public void init(byte[] image) {
        _isRunning = true;
        // Se asigna la referencia a la clase EKF(C++) desde la funcion nativa
        // Y va a quedar en _ekfNativeReference
        loadEKFNativeReference(_configFilePath, _outputPath);
        
        // Se cargan en las variables de la clase las dimensiones para las que va a trabajar el EKF
        loadEKFCameraPreviewDimensions();
        
        EKFInit(image, _cameraPreviewWidth, _cameraPreviewHeight);
        image = null;
        
        _isRunning = false;
    }
    
    public void step(byte[] image) {
        _isRunning = true;
        
        EKFStep(image, _cameraPreviewWidth, _cameraPreviewHeight);
        image = null;
        
        _isRunning = false;
    }
    
    public void release() {
        releaseEKFNativeReference();
    }

    public double getxCam() {
        return xCam;
    }

    public double getyCam() {
        return yCam;
    }

    public double getzCam() {
        return zCam;
    }
    
    public int getCameraPreviewWidth() {
        return _cameraPreviewWidth;
    }
    public int getCameraPreviewHeight() {
        return _cameraPreviewHeight;
    }
    
    public boolean isRunning() {
        return _isRunning;
    }
    
    public void set_isRunning(boolean is) {
        _isRunning = is;
    }
}
