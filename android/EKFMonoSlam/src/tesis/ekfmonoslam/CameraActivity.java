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

import java.io.File;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Intent;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

@SuppressLint("DefaultLocale")
public class CameraActivity extends Activity {
    // ---------------------------------------------------------------------------------------
    // Variables locales
    // ---------------------------------------------------------------------------------------
    private String TAG = "CameraActivity";
    private Button _cameraStartEKFButton;
    private CheckBox _previewCheckBox;
    private CameraManager _camera;
    private int _steps = 0;
    private String _programFolder;
    private String _configurationPath;
    private String _configurationFilePath;
    private String _outputPath; 
    private int _cameraPreviewWidth = 640;
    private int _cameraPreviewHeight = 480;
    private EKF _ekf;
    private TextView _xCam;
    private TextView _yCam;
    private TextView _zCam;
    private boolean _isEKFprocessRunning = false;
    private ImageView _ekfPredictionView;
    private byte[] _previewImageData = null;
    
    private ThreadPoolExecutor _threadPool = null;
    private int _poolSize = 10;
    private int _maxPoolSize = 20;
    private long _keepAliveTime = 10;
    // ---------------------------------------------------------------------------------------
    // Metodos de clase
    // ---------------------------------------------------------------------------------------
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        _cameraStartEKFButton = (Button)findViewById(R.id.button_start_stop_ekf);
        _cameraStartEKFButton.setOnClickListener(_startStopEKFButtonListener);

        _previewCheckBox = (CheckBox)findViewById(R.id.check_box_preview);
        // Aca se va a mostrar la posicion de la camara
        _xCam = (TextView)findViewById(R.id.xCam);
        _yCam = (TextView)findViewById(R.id.yCam);
        _zCam = (TextView)findViewById(R.id.zCam);
        
        // Aca se van a mostrar las predicciones y las elipses dentro del frame actual.
        _ekfPredictionView = (ImageView)findViewById(R.id.ekf_prediction_view);
        
        _camera = (CameraManager)findViewById(R.id.cameraManager);
        
        // Directorios del programa
        Resources res = getResources();
        _programFolder = res.getString(R.string.program_folder);
        _configurationPath = Environment.getExternalStorageDirectory().getPath() + "/" + _programFolder + "/";
        _configurationFilePath = _configurationPath + res.getString(R.string.configuration_file);
        _outputPath = _configurationPath + res.getString(R.string.output_folder) + "/";
    }

    // ---------------------------------------------------------------------------------------

    @Override
    protected void onResume() {
        super.onResume();
        
        _ekf = new EKF(_configurationFilePath, _outputPath, _cameraPreviewWidth, _cameraPreviewHeight);
        
        _threadPool = new ThreadPoolExecutor(_poolSize, _maxPoolSize, _keepAliveTime, TimeUnit.SECONDS, new ArrayBlockingQueue<Runnable>(_poolSize));
        
        _camera.setPreviewSize(_ekf.getCameraPreviewWidth(), _ekf.getCameraPreviewHeight());
    }

    // ---------------------------------------------------------------------------------------

    @Override
    protected void onPause() {
        _isEKFprocessRunning = false;
        
        _camera.setPreviewCallback(null);
        
        _threadPool.shutdown();
        
        _ekf.release();
        _ekf = null;

        // Se actualiza la libreria de fotos
        sendBroadcast(new Intent(Intent.ACTION_MEDIA_MOUNTED, Uri.parse("file://" + Environment.getExternalStorageDirectory())));
        
        super.onPause();
    }

    // ---------------------------------------------------------------------------------------

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.activity_main, menu);
        return true;
    }

    // ---------------------------------------------------------------------------------------

    final private View.OnClickListener _startStopEKFButtonListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            v = null;
            
            if (_isEKFprocessRunning) {
                _cameraStartEKFButton.setText(R.string.button_start_ekf);
                _isEKFprocessRunning = !_isEKFprocessRunning;
                _camera.setPreviewCallback(null);
                
                Toast.makeText(getApplicationContext(), "Proceso finalizado.", Toast.LENGTH_SHORT).show();
            } else {
                _isEKFprocessRunning = true;
                _camera.setPreviewCallback(_cameraPreviewCallback);
                _threadPool.execute(new EKFTask());
                
                _cameraStartEKFButton.setText(R.string.button_stop_ekf);
            }
        }
    };

    // ---------------------------------------------------------------------------------------
    
    private PreviewCallback _cameraPreviewCallback = new PreviewCallback() {
        
        @Override
        public void onPreviewFrame(byte[] data, Camera camera) {
            // Log.e(TAG, "Se esta llamando onPreviewFrame!");
            //if (!_ekf.isRunning()) {
                _previewImageData = data;
            //}
            
            data = null;
        }
    };
    
    // ---------------------------------------------------------------------------------------

    private CameraManager.Callback _cameraCallback = new CameraManager.Callback(){
        @Override
        public void onAutoFocusFinished(boolean success) {
        }

        @Override
        public void onPictureTaken(Bitmap photo) {
        }

        @Override
        public void onPreviewFrameTaken(byte[] data) {
            //Log.e(TAG, "Se está llamando a onPreviewFrameTaken");
            
            _previewImageData = data;
            
            data = null;
        }
    };

    // ---------------------------------------------------------------------------------------
    public void setCameraPosition() {
        _ekf.getxCam();
        _ekf.getyCam();
        _ekf.getzCam();
        
        _xCam.setText(Double.toString(_ekf.getxCam()));
        _yCam.setText(Double.toString(_ekf.getyCam()));
        _zCam.setText(Double.toString(_ekf.getzCam()));
    }
    
    //-------------------------------------------------------------------------------------------------------
    // Clase que pide frames a la camara para que ejecute los pasos del EKF
    //-------------------------------------------------------------------------------------------------------
    private class EKFTask implements Runnable {
        
        public void run() {
            while (_previewImageData == null) {
                Thread.currentThread();
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            
            while (_isEKFprocessRunning) {
                _ekf.set_isRunning(true);
                
                if (_steps < 1) {
                    _ekf.init(_previewImageData);
                } else {
                    _ekf.step(_previewImageData);
                    
                    if (_previewCheckBox.isChecked()) {
                        _threadPool.execute(new PainterTask(_steps));
                    }
                }
                
                _steps++;
            }
        }
    }
    
    //-------------------------------------------------------------------------------------------------------
    // Clase que se encarga de pintar los frames con las elipses de la prediccion
    //-------------------------------------------------------------------------------------------------------
    
    private class PainterTask implements Runnable {
        private String TAG = "PainterTask";
        private Bitmap image = null;
        private int _step = 0;
        
        public PainterTask(int step) {
            _step = step;
        }
        
        public void run() {
            String imageName = String.format("%05d.png", _step);
            String path = _outputPath + imageName;
            Log.i(TAG, "Se quiere abrir en " + path);
            
            
            while (!new File(_outputPath + imageName).exists() && _isEKFprocessRunning) {
                Log.i(TAG, "No existe el archivo " + path);
                Thread.currentThread();
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            
            if (_isEKFprocessRunning) {
                image = BitmapFactory.decodeFile(_outputPath + imageName);
            }
            
            runOnUiThread(new Runnable() {
                
                @Override
                public void run() {
                    _ekfPredictionView.setImageBitmap(image);
                    setCameraPosition();
                }
            });
        }
    }
    
    private class PreviewRequest implements Runnable {
        public void run() {
            while (_isEKFprocessRunning) {
                _camera.captureNextFrame(_cameraCallback);
               
               Thread.currentThread();
               try {
                   Thread.sleep(1);
               } catch (InterruptedException e) {
                   e.printStackTrace();
               }
            }
        }
    }
}
