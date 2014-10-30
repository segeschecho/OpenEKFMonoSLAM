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

import android.app.Activity;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

public class ImageSequenceActivity extends Activity{
    private Button _startButton;
    private boolean _isRunning = false;
    private TextView _pathView;
    private ImageView _actualImageView;
    private CheckBox _previewCheckBox;
    private CheckBox _onDemandCheckBox;
    
    private EKF _ekf;
    
    private String TAG = "ImageSequenceActivity";
    
    private String _programFolder;
    private String _configurationPath;
    private String _configurationFilePath;
    private String _outputPath;
    private String _inputPath;
    
    // ---------------------------------------------------------------------------------------
    // Metodos de clase
    // ---------------------------------------------------------------------------------------
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_image_sequence);
        
        _startButton = (Button)findViewById(R.id.button_process_images);
        _startButton.setOnClickListener(_startButtonListener);
        
        _previewCheckBox = (CheckBox)findViewById(R.id.preview_image_sequence);
        _onDemandCheckBox = (CheckBox)findViewById(R.id.check_box_on_demand);
        
        Resources res = getResources();
        _programFolder = res.getString(R.string.program_folder);
        _configurationPath = Environment.getExternalStorageDirectory().getPath() + "/" + _programFolder + "/";
        _configurationFilePath = _configurationPath + res.getString(R.string.configuration_file);
        _outputPath = _configurationPath + res.getString(R.string.output_folder) + "/";
        _inputPath = _configurationPath + res.getString(R.string.input_folder) + "/";
        
        _pathView = (TextView)findViewById(R.id.textView1);
        _pathView.setText(_inputPath);
        
        _actualImageView = (ImageView)findViewById(R.id.imageView1);
    }
    
    // ---------------------------------------------------------------------------------------

    @Override
    protected void onResume() {
        super.onResume();

        _ekf = new EKF(_configurationFilePath, _outputPath, 640, 480);
    }

    // ---------------------------------------------------------------------------------------

    @Override
    protected void onPause() {
        _ekf.release();
        _ekf = null;
        
        super.onPause();
    }
    
    final private View.OnClickListener _startButtonListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            if (_isRunning) {
                // Se finaliza el proceso
                _startButton.setText(R.string.button_process_images);
                _isRunning = false;
            } else {
                // Se comienza a procesar las imagenes
                _startButton.setText("Stop");
                _isRunning = true;
                
                new ImageProcessor().execute();
            }
        }
    };
    
    
    private class ImageProcessor extends AsyncTask<Void, Bitmap, Void> {
        String TAG = "ImageProcessor";
        
        private int frameRate = 30;
        
        protected Void doInBackground(Void... params) {
            
            int imageIndex = 1;
            boolean hasImage = true;
            
            long initialTime = 0;
            long finalTime = 0;
            float elapsedTime = 0;
            int framesToSetForward = 0;
            
            while (_isRunning && hasImage) {
                String imageName = String.format("%05d.png", imageIndex);
                String path = _inputPath + imageName;
                
                Log.i(TAG, "Se quiere abrir la imagen: " + path);
                
                Bitmap image = BitmapFactory.decodeFile(path);
                
                if (image == null) {
                    Log.i(TAG, "No se encontro la imagen en: " + path);
                    hasImage = false;
                    break;
                }
                
                int mWidth = image.getWidth();
                int mHeight = image.getHeight();

                int[] pixels = new int[mWidth * mHeight];
                byte[] b = new byte[mWidth*mHeight*3/2];
                image.getPixels(pixels, 0, mWidth, 0, 0, mWidth, mHeight);
                encodeYUV420SP(b, pixels, mWidth, mHeight);
                
                if (_onDemandCheckBox.isChecked()) {
                    initialTime = System.currentTimeMillis();
                }
                
                if (imageIndex <= 1) {
                    _ekf.init(b);
                } else {
                    _ekf.step(b);
                }
                
                if (_onDemandCheckBox.isChecked()) {
                    finalTime = System.currentTimeMillis();
                    elapsedTime = (finalTime - initialTime)/1000.0f;
                    framesToSetForward = (int)(elapsedTime * frameRate);
                    Log.i(TAG, "elapsedTime: " + elapsedTime + " framesToSetFor: " + framesToSetForward);
                    imageIndex += framesToSetForward;
                } else {
                    imageIndex++;
                }
                
                if (_previewCheckBox.isChecked()) {
                    publishProgress(image);
                }
                
                
            }
            
            return null;
        }
        
        protected void onProgressUpdate(Bitmap... params) {
            _actualImageView.setImageBitmap(params[0]);
        }
        
        protected void onPostExecute(Void p) {
            Toast.makeText(getApplicationContext(), "Proceso finalizado.", Toast.LENGTH_SHORT).show();
        }
        
        private void encodeYUV420SP(byte[] yuv420sp, int[] argb, int width, int height) {
            final int frameSize = width * height;

            int yIndex = 0;
            int uvIndex = frameSize;

            int a, R, G, B, Y, U, V;
            int index = 0;
            for (int j = 0; j < height; j++) {
                for (int i = 0; i < width; i++) {

                    a = (argb[index] & 0xff000000) >> 24; // a is not used obviously
                    R = (argb[index] & 0xff0000) >> 16;
                    G = (argb[index] & 0xff00) >> 8;
                    B = (argb[index] & 0xff) >> 0;

                    // well known RGB to YUV algorithm
                    Y = ( (  66 * R + 129 * G +  25 * B + 128) >> 8) +  16;
                    U = ( ( -38 * R -  74 * G + 112 * B + 128) >> 8) + 128;
                    V = ( ( 112 * R -  94 * G -  18 * B + 128) >> 8) + 128;

                    // NV21 has a plane of Y and interleaved planes of VU each sampled by a factor of 2
                    //    meaning for every 4 Y pixels there are 1 V and 1 U.  Note the sampling is every other
                    //    pixel AND every other scanline.
                    yuv420sp[yIndex++] = (byte) ((Y < 0) ? 0 : ((Y > 255) ? 255 : Y));
                    if (j % 2 == 0 && index % 2 == 0) { 
                        yuv420sp[uvIndex++] = (byte)((V<0) ? 0 : ((V > 255) ? 255 : V));
                        yuv420sp[uvIndex++] = (byte)((U<0) ? 0 : ((U > 255) ? 255 : U));
                    }

                    index ++;
                }
            }
        }
    }
}
