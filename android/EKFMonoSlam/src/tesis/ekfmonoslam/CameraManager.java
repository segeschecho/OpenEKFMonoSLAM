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

import java.io.IOException;
import java.util.List;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.hardware.Camera;
import android.hardware.Camera.AutoFocusCallback;
import android.hardware.Camera.PictureCallback;
import android.hardware.Camera.PreviewCallback;
import android.hardware.Camera.Size;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.ViewGroup.LayoutParams;

public class CameraManager extends SurfaceView implements SurfaceHolder.Callback {

    public interface Callback {
        void onAutoFocusFinished(boolean success);
        void onPictureTaken(Bitmap photo);
        void onPreviewFrameTaken(byte[] data);
    }

    private final String TAG = "CameraManager";

    SurfaceHolder 	_holder;
    int				_measuredDimensionWidth;
    int				_measuredDimensionHeight;
    List<Size> 		_supportedPreviewSizes;
    Camera 			_camera;

    CameraManager.Callback _callback;

    Integer			_onPictureBoundWidth;
    Integer			_onPictureBoundHeight;

    int _previewSizeWidth = 0;
    int _previewSizeHeight = 0;
    // --------------------------------------------------------------------------
    // Constructors

    public CameraManager(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        init(context);
    }

    // --------------------------------------------------------------------------

    public CameraManager(Context context, AttributeSet attrs) {
        super(context, attrs);
        init(context);
    }

    // --------------------------------------------------------------------------

    CameraManager(Context context) {
        super(context);    
        init(context);
    }

    // --------------------------------------------------------------------------

    void init(Context context) {
        // Install a SurfaceHolder.Callback so we get notified when the
        // underlying surface is created and destroyed.
        _holder = getHolder();
        _holder.addCallback(this);
        _holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
    }

    // --------------------------------------------------------------------------
    // Methods

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        _measuredDimensionWidth = resolveSize(getSuggestedMinimumWidth(), widthMeasureSpec);
        _measuredDimensionHeight = resolveSize(getSuggestedMinimumHeight(), heightMeasureSpec);
        setMeasuredDimension(_measuredDimensionWidth, _measuredDimensionHeight);
    }

    // --------------------------------------------------------------------------

    public void surfaceCreated(SurfaceHolder holder) {
        // The Surface has been created, acquire the camera and tell it where
        // to draw.
        try {
            _camera = Camera.open();

            if (_camera != null) {
                _supportedPreviewSizes = _camera.getParameters().getSupportedPreviewSizes();
                List<Size> supportedPictureSizes = _camera.getParameters().getSupportedPictureSizes();

                if (_previewSizeHeight == 0 || _previewSizeWidth == 0) {
                    Size pictureSize = supportedPictureSizes.get(0);
                    Size previewSize = getOptimalPreviewSize(_supportedPreviewSizes, pictureSize.width, pictureSize.height);
                    
                    _previewSizeHeight = previewSize.height;
                    _previewSizeWidth = previewSize.width;
                }
                
                // Resize view in order to respect ratio		
                LayoutParams resizedParams = getLayoutParams();
                double ratio = (double) _previewSizeWidth / _previewSizeHeight;
                resizedParams.height = (int) Math.min(_measuredDimensionWidth * 1.0/ratio, _measuredDimensionHeight);
                resizedParams.width = (int) Math.min(_measuredDimensionHeight * ratio, _measuredDimensionWidth); 
                setLayoutParams(resizedParams);

                _camera.setPreviewDisplay(holder);
            }

        } catch (IOException exception) {
            Log.e(TAG, "IOException caused by setPreviewDisplay()", exception);
        }
    }

    // --------------------------------------------------------------------------

    public void surfaceDestroyed(SurfaceHolder holder) {
        // Surface will be destroyed when we return, so stop the preview.
        if (_camera != null) {
            _camera.stopPreview();
            _camera.release();
            _camera = null;
        }

        _callback = null;
    }

    // --------------------------------------------------------------------------

    private Size getOptimalPreviewSize(List<Size> sizes, int w, int h) {
        final double ASPECT_TOLERANCE = 0.1;
        double targetRatio = (double) w / h;
        if (sizes == null) return null;

        Size optimalSize = null;
        double minDiff = Double.MAX_VALUE;

        int targetHeight = h;

        // Try to find an size match aspect ratio and size
        for (Size size : sizes) {
            double ratio = (double) size.width / size.height;
            if (Math.abs(ratio - targetRatio) > ASPECT_TOLERANCE) continue;
            if (Math.abs(size.height - targetHeight) < minDiff) {
                optimalSize = size;
                minDiff = Math.abs(size.height - targetHeight);
            }
        }

        // Cannot find the one match the aspect ratio, ignore the requirement
        if (optimalSize == null) {
            minDiff = Double.MAX_VALUE;
            for (Size size : sizes) {
                if (Math.abs(size.height - targetHeight) < minDiff) {
                    optimalSize = size;
                    minDiff = Math.abs(size.height - targetHeight);
                }
            }
        }
        return optimalSize;
    }

    // --------------------------------------------------------------------------

    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
        // Now that the size is known, set up the camera parameters and begin
        // the preview.
        Camera.Parameters parameters = _camera.getParameters();
        parameters.setPreviewSize(_previewSizeWidth, _previewSizeHeight);
        requestLayout();

        Log.d(TAG, "Preview Size: " + parameters.getPreviewSize().width + " x " + parameters.getPreviewSize().height);
        _camera.setParameters(parameters);
        _camera.startPreview();
    }

    public void setPreviewSize(int width, int height) {
        _previewSizeWidth = width;
        _previewSizeHeight = height;
    }
    
    // --------------------------------------------------------------------------

    public void takePicture(CameraManager.Callback callback) {
        if( _callback != null ) {
            Log.d(TAG, "CALLBACK should be NULL!");

            // let's avoid the app to crash
            return;
        }

        if(_camera == null) {
            callback.onPictureTaken(null);
            return;
        }

        _callback = callback;
        _camera.takePicture(null, null, _pictureCallback);
    }

    // --------------------------------------------------------------------------

    public void takePicture(CameraManager.Callback callback, int boundWidth, int boundHeight) {
        _onPictureBoundWidth = boundWidth;
        _onPictureBoundHeight = boundHeight;

        takePicture(callback);
    }

    // --------------------------------------------------------------------------

    public void focusCamera(CameraManager.Callback callback) {
        if( _callback != null ) {
            Log.d(TAG, "CALLBACK should be NULL!");

            // let's avoid the app to crash
            return;
        }

        if(_camera == null) {
            callback.onAutoFocusFinished(false);
            return;
        }

        _callback = callback;
        _camera.autoFocus(_autoFocusCallback);    		
    }

    // --------------------------------------------------------------------------
    
    public void setPreviewCallback(PreviewCallback callback) {
        if (_camera != null) {
            _camera.setPreviewCallback(callback);
        }
    }
    
    // --------------------------------------------------------------------------

    final private AutoFocusCallback _autoFocusCallback = new AutoFocusCallback() {

        public void onAutoFocus(boolean success, Camera camera) {
            Log.d(TAG, "Auto focus finished...");

            if (_callback != null) {
                CameraManager.Callback callback = _callback;
                _callback = null;
                callback.onAutoFocusFinished(success);
            }
        }
    };

    // --------------------------------------------------------------------------

    final private PictureCallback _pictureCallback = new PictureCallback() {
        public void onPictureTaken(byte[] data, Camera camera) {
            Log.d(TAG, String.format("Picture taken length = %d", data.length));

            BitmapFactory.Options options = null;

            if (_onPictureBoundWidth != null && _onPictureBoundHeight != null) {
                BitmapFactory.Options justDecodeBounds = new BitmapFactory.Options();
                justDecodeBounds.inJustDecodeBounds = true;

                BitmapFactory.decodeByteArray(data, 0, data.length, justDecodeBounds);

                options = new BitmapFactory.Options();

                // Ensure the bitmap won't exceed bounds, setting inSampleSize
                if (justDecodeBounds.outHeight > _onPictureBoundHeight || justDecodeBounds.outWidth > _onPictureBoundWidth) {
                    if (justDecodeBounds.outWidth > justDecodeBounds.outHeight) {
                        options.inSampleSize = Math.round((float)justDecodeBounds.outHeight / (float)_onPictureBoundHeight);
                    } else {
                        options.inSampleSize = Math.round((float)justDecodeBounds.outWidth / (float)_onPictureBoundWidth);
                    }
                }

                _onPictureBoundWidth = null;
                _onPictureBoundHeight = null;
            }

            Bitmap bitmap = BitmapFactory.decodeByteArray(data, 0, data.length, options);

            if (_callback != null) {
                CameraManager.Callback callback = _callback;
                _callback = null;
                callback.onPictureTaken(bitmap);

                if (_camera != null) {
                    _camera.startPreview();
                }
            }
        }
    };

    // --------------------------------------------------------------------------

    public void captureNextFrame(CameraManager.Callback callback) {
        if (_camera != null) {
            _callback = callback;
            _camera.setOneShotPreviewCallback(_previewCallback);
        }
    }

    // --------------------------------------------------------------------------

    final private PreviewCallback _previewCallback = new PreviewCallback() {		
        public void onPreviewFrame(byte[] data, Camera camera) {
            // Log.i(TAG, "onPreviewFrame called!");
            _callback.onPreviewFrameTaken(data);

            data = null;
        }
    };
}