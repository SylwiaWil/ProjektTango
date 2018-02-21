/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.examples.java.pointcloud;

import android.app.Activity;
import android.content.Context;
import android.hardware.display.DisplayManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.google.tango.support.TangoPointCloudManager;
import com.google.tango.support.TangoSupport;
import com.google.tango.ux.TangoUx;
import com.google.tango.ux.UxExceptionEvent;
import com.google.tango.ux.UxExceptionEventListener;

import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.scene.ASceneFrameCallback;
import org.rajawali3d.surface.RajawaliSurfaceView;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.text.DecimalFormat;
import java.util.ArrayList;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the {@link Tango}
 * service and propagation of Tango point cloud data to OpenGL and Layout views. OpenGL rendering
 * logic is delegated to the {@link PointCloudRajawaliRenderer} class.
 */
public class PointCloudActivity extends Activity {
    private static final String TAG = PointCloudActivity.class.getSimpleName();

    private static final String UX_EXCEPTION_EVENT_DETECTED = "Exception Detected: ";
    private static final String UX_EXCEPTION_EVENT_RESOLVED = "Exception Resolved: ";

    private static final int SECS_TO_MILLISECS = 1000;
    private static final DecimalFormat FORMAT_THREE_DECIMAL = new DecimalFormat("0.000");
    private static final double UPDATE_INTERVAL_MS = 100.0;

    private Tango mTango;
    private TangoConfig mConfig;
    private TangoUx mTangoUx;


    private TangoPointCloudManager mPointCloudManager;
    private PointCloudRajawaliRenderer mRenderer;
    private RajawaliSurfaceView mSurfaceView;
    private TextView mPointCountTextView;

    private TextView mAverageZTextView;
    private double mPointCloudPreviousTimeStamp;

    private boolean mIsConnected = false;

    private double mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;

    private int mDisplayRotation = 0;
    private Button freezeBtn;
    private Button saveBtn;
    private boolean isFreeze = false;

    private static final String mMainDirPath = Environment.getExternalStorageDirectory()
            .getAbsolutePath() + "/Tango/";
    private static final String mSaveDirAbsPath = Environment.getExternalStorageDirectory()
            .getAbsolutePath() + "/Tango/MyPointCloudData/";
    private String mFilename;
    private int mNumberOfFilesWritten;
    private Boolean mTimeToTakeSnap;
    private Boolean mAutoMode;
    private String mNowTimeString;
    private ArrayList<String> mFilenameBuffer;
    private float mCurrentTimeStamp;
    private float mPosePreviousTimeStamp;
    private float mXyIjPreviousTimeStamp;
    private int mXyzIjCallbackCount;
    FileOutputStream outputStream;
    String filename = "pointCloud";
    TangoPointCloudData pointCloud;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_point_cloud);

        mPointCountTextView = (TextView) findViewById(R.id.point_count_textview);
        mAverageZTextView = (TextView) findViewById(R.id.average_z_textview);
        mSurfaceView = (RajawaliSurfaceView) findViewById(R.id.gl_surface_view);
        freezeBtn = (Button) findViewById(R.id.freeze_btn);
        saveBtn = (Button) findViewById(R.id.save_btn);

        mPointCloudManager = new TangoPointCloudManager();
        mTangoUx = setupTangoUxAndLayout();
        mRenderer = new PointCloudRajawaliRenderer(this);
        setupRenderer();

        DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {
                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        setDisplayRotation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {
                }
            }, null);
        }
    }


    @Override
    protected void onResume() {
        super.onResume();
        mFilename = "";
        mNumberOfFilesWritten = 0;
        mTimeToTakeSnap = false;
        mAutoMode = false;
        mFilenameBuffer = new ArrayList<String>();

        freezeBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (isFreeze) {
                    mRenderer.startRendering();
                    pointCloud = mPointCloudManager.getLatestPointCloud();
                    isFreeze = false;
                } else {
                    mRenderer.stopRendering();
                    isFreeze = true;
                }
            }
        });

        saveBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                try {
                    File path = Environment.getExternalStoragePublicDirectory(
                            Environment.DIRECTORY_DOWNLOADS);
                    File myFile = new File(path, "mytextfile.csv");
                    FileOutputStream fileout = new FileOutputStream(myFile, true);
                    OutputStreamWriter outputWriter = new OutputStreamWriter(fileout);
                   /* outputWriter.write("PointCloudInfo");
                    outputWriter.write(mRenderer.mPointCloud.getGeometry().toString());*/

                    float[] points = mRenderer.mPointCloud.getPoint(pointCloud.numPoints, pointCloud.points);
                    for (int i = 0; i < points.length - 2; i++) {
                        outputWriter.write(points[i] + ";");
                        outputWriter.write(points[i + 1] + ";");
                        outputWriter.write(points[i + 2] + ";" + "\n");

                    }
                    // outputWriter.write(Arrays.toString(points));

                    outputWriter.close();


                    //display file saved message
                    Toast.makeText(getBaseContext(), "File saved successfully!",
                            Toast.LENGTH_SHORT).show();

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
    }

    private void setTangoListeners() {
        // Configure the Tango coordinate frame pair
        final ArrayList<TangoCoordinateFramePair> framePairs =
                new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        // Listen for new Tango data
        mTango.connectListener(framePairs, new Tango.OnTangoUpdateListener() {

            public TangoXyzIjData xyzIj;

            @Override
            public void onPoseAvailable(final TangoPoseData pose) {

                // Update the UI with TangoPose information
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        DecimalFormat threeDec = new DecimalFormat("0.000");
                        String translationString = "["
                                + threeDec.format(pose.translation[0]) + ", "
                                + threeDec.format(pose.translation[1]) + ", "
                                + threeDec.format(pose.translation[2]) + "] ";
                        String quaternionString = "["
                                + threeDec.format(pose.rotation[0]) + ", "
                                + threeDec.format(pose.rotation[1]) + ", "
                                + threeDec.format(pose.rotation[2]) + ", "
                                + threeDec.format(pose.rotation[3]) + "] ";
                    }
                });
            }

            @Override
            public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
                mCurrentTimeStamp = (float) xyzIj.timestamp;
                final float frameDelta = (mCurrentTimeStamp - mXyIjPreviousTimeStamp) * SECS_TO_MILLISECS;
                mXyIjPreviousTimeStamp = mCurrentTimeStamp;
                mXyzIjCallbackCount++;

                final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
                final byte[] buffer = new byte[xyzIj.xyzCount * 3 * 4];
                FileInputStream fileStream = new FileInputStream(
                        xyzIj.xyzParcelFileDescriptor.getFileDescriptor());
                try {
                    fileStream.read(buffer, xyzIj.xyzParcelFileDescriptorOffset, buffer.length);
                    fileStream.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
                writePointCloudToFile(xyzIj, buffer, framePairs);
            }

        });
    }


    private void writePointCloudToFile(TangoXyzIjData xyzIj, byte[] buffer,
                                       ArrayList<TangoCoordinateFramePair> framePairs) {

        ByteBuffer myBuffer = ByteBuffer.allocate(xyzIj.xyzCount * 3 * 4);
        myBuffer.order(ByteOrder.LITTLE_ENDIAN);
        myBuffer.put(buffer, xyzIj.xyzParcelFileDescriptorOffset, myBuffer.capacity());


        File mainDir = new File(mMainDirPath);
        if (!mainDir.exists()) {
            boolean created = mainDir.mkdir();
            if (created) {
                Log.i(TAG, "Folder: \"" + mMainDirPath + "\" created\n");
            }
        }

        File dir = new File(mSaveDirAbsPath);
        if (!dir.exists()) {
            boolean created = dir.mkdir();
            if (created) {
                Log.i(TAG, "Folder: \"" + mSaveDirAbsPath + "\" created\n");
            }
        }

        mFilename = "pc_" + mNowTimeString + "_" + String.format("%03d", mNumberOfFilesWritten) +
                ".vtk";
        mFilenameBuffer.add(mSaveDirAbsPath + mFilename);
        File file = new File(dir, mFilename);


        try {

            DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
                    new FileOutputStream(file)));

            out.write(("# vtk DataFile Version 3.0\n" +
                    "vtk output\n" +
                    "BINARY\n" +
                    "DATASET POLYDATA\n" +
                    "POINTS " + xyzIj.xyzCount + " float\n").getBytes());

            for (int i = 0; i < xyzIj.xyzCount; i++) {

                out.writeFloat(myBuffer.getFloat(3 * i * 4));
                out.writeFloat(myBuffer.getFloat((3 * i + 1) * 4));
                out.writeFloat(myBuffer.getFloat((3 * i + 2) * 4));
            }

            out.write(("\nVERTICES 1 " + String.valueOf(xyzIj.xyzCount + 1) + "\n").getBytes());
            out.writeInt(xyzIj.xyzCount);
            for (int i = 0; i < xyzIj.xyzCount; i++) {
                out.writeInt(i);
            }

            out.write(("\nFIELD FieldData 1\n" + "timestamp 1 1 float\n").getBytes());
            out.writeFloat((float) xyzIj.timestamp);

            out.close();
            mNumberOfFilesWritten++;
            mTimeToTakeSnap = false;

        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    protected void onStart() {
        super.onStart();

        mTangoUx.start();
        bindTangoService();
    }

    @Override
    protected void onStop() {
        super.onStop();

        // Synchronize against disconnecting while the service is being used in the OpenGL
        // thread or in the UI thread.
        // NOTE: DO NOT lock against this same object in the Tango callback thread.
        // Tango.disconnect will block here until all Tango callback calls are finished.
        // If you lock against this object in a Tango callback thread it will cause a deadlock.
        synchronized (this) {
            try {
                mTangoUx.stop();
                mTango.disconnect();
                mIsConnected = false;
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }
    }

    /**
     * Initialize Tango Service as a normal Android Service.
     */
    private void bindTangoService() {
        // Initialize Tango Service as a normal Android Service. Since we call mTango.disconnect()
        // in onPause, this will unbind Tango Service, so every time onResume gets called we
        // should create a new Tango object.
        mTango = new Tango(PointCloudActivity.this, new Runnable() {
            // Pass in a Runnable to be called from UI thread when Tango is ready; this Runnable
            // will be running on a new thread.
            // When Tango is ready, we can call Tango functions safely here only when there are no
            // UI thread changes involved.
            @Override
            public void run() {
                // Synchronize against disconnecting while the service is being used in the OpenGL
                // thread or in the UI thread.
                synchronized (PointCloudActivity.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);
                        startupTango();
                        TangoSupport.initialize(mTango);
                        mIsConnected = true;
                        setDisplayRotation();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_error);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_invalid);
                    }
                }
            }
        });
    }

    /**
     * Sets up the Tango configuration object. Make sure mTango object is initialized before
     * making this call.
     */
    private TangoConfig setupTangoConfig(Tango tango) {
        // Use the default configuration plus add depth sensing.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        return config;
    }

    /**
     * Set up the callback listeners for the Tango Service and obtain other parameters required
     * after Tango connection.
     * Listen to updates from the Point Cloud and Tango Events and Pose.
     */
    private void startupTango() {
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();

        framePairs.add(new TangoCoordinateFramePair(TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                // Passing in the pose data to UX library produce exceptions.
                if (mTangoUx != null) {
                    mTangoUx.updatePoseStatus(pose.statusCode);
                }
            }

            @Override
            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {
                if (mTangoUx != null) {
                    mTangoUx.updatePointCloud(pointCloud);
                }
                mPointCloudManager.updatePointCloud(pointCloud);

                final double currentTimeStamp = pointCloud.timestamp;
                final double pointCloudFrameDelta =
                        (currentTimeStamp - mPointCloudPreviousTimeStamp) * SECS_TO_MILLISECS;
                mPointCloudPreviousTimeStamp = currentTimeStamp;
                final double averageDepth = getAveragedDepth(pointCloud.points,
                        pointCloud.numPoints);

                mPointCloudTimeToNextUpdate -= pointCloudFrameDelta;

                if (mPointCloudTimeToNextUpdate < 0.0) {
                    mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;
                    final String pointCountString = Integer.toString(pointCloud.numPoints);
                    TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
                    //normalize(pointCloud, mTango.getPoseAtTime(0.0, framePair), setupExtrinsics(mTango));
                    // savePointCloud(pointCloud);

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mPointCountTextView.setText(pointCountString);
                            mAverageZTextView.setText(FORMAT_THREE_DECIMAL.format(averageDepth));
                        }
                    });
                }
            }

            @Override
            public void onTangoEvent(TangoEvent event) {
                if (mTangoUx != null) {
                    mTangoUx.updateTangoEvent(event);
                }
            }
        });
    }


    /**
     * Sets Rajawali surface view and its renderer. This is ideally called only once in onCreate.
     */
    public void setupRenderer() {
        mSurfaceView.setEGLContextClientVersion(2);
        mRenderer.getCurrentScene().registerFrameCallback(new ASceneFrameCallback() {
            @Override
            public void onPreFrame(long sceneTime, double deltaTime) {
                // NOTE: This will be executed on each cycle before rendering; called from the
                // OpenGL rendering thread.

                // Prevent concurrent access from a service disconnect through the onPause event.
                synchronized (PointCloudActivity.this) {
                    // Don't execute any Tango API actions if we're not connected to the service.
                    if (!mIsConnected) {
                        return;
                    }

                    // Update point cloud data.
                    TangoPointCloudData pointCloud = mPointCloudManager.getLatestPointCloud();
                    if (pointCloud != null) {
                        // Calculate the depth camera pose at the last point cloud update.
                        TangoSupport.MatrixTransformData transform =
                                TangoSupport.getMatrixTransformAtTime(pointCloud.timestamp,
                                        TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                        TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                        TangoSupport.ENGINE_OPENGL,
                                        TangoSupport.ENGINE_TANGO,
                                        TangoSupport.ROTATION_IGNORED);
                        if (transform.statusCode == TangoPoseData.POSE_VALID) {
                            mRenderer.updatePointCloud(pointCloud, transform.matrix);
                        }
                    }

                    // Update current camera pose.
                    try {
                        // Calculate the device pose. This transform is used to display
                        // frustum in third and top down view, and used to render camera pose in
                        // first person view.
                        TangoPoseData lastFramePose = TangoSupport.getPoseAtTime(0,
                                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                TangoPoseData.COORDINATE_FRAME_DEVICE,
                                TangoSupport.ENGINE_OPENGL,
                                TangoSupport.ENGINE_OPENGL,
                                mDisplayRotation);
                        if (lastFramePose.statusCode == TangoPoseData.POSE_VALID) {
                            mRenderer.updateCameraPose(lastFramePose);
                        }
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "Could not get valid transform");
                    }
                }
            }

            @Override
            public boolean callPreFrame() {
                return true;
            }

            @Override
            public void onPreDraw(long sceneTime, double deltaTime) {

            }

            @Override
            public void onPostFrame(long sceneTime, double deltaTime) {

            }
        });
        mSurfaceView.setSurfaceRenderer(mRenderer);
    }

    /**
     * Sets up TangoUX and sets its listener.
     */
    private TangoUx setupTangoUxAndLayout() {
        TangoUx tangoUx = new TangoUx(this);
        tangoUx.setUxExceptionEventListener(mUxExceptionListener);
        return tangoUx;
    }

    /*
    * Set a UxExceptionEventListener to be notified of any UX exceptions.
    * In this example we are just logging all the exceptions to logcat, but in a real app,
    * developers should use these exceptions to contextually notify the user and help direct the
    * user in using the device in a way Tango Service expects it.
    * <p>
    * A UxExceptionEvent can have two statuses: DETECTED and RESOLVED.
    * An event is considered DETECTED when the exception conditions are observed, and RESOLVED when
    * the root causes have been addressed.
    * Both statuses will trigger a separate event.
    */
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {
        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {
            String status = uxExceptionEvent.getStatus() == UxExceptionEvent.STATUS_DETECTED ?
                    UX_EXCEPTION_EVENT_DETECTED : UX_EXCEPTION_EVENT_RESOLVED;

            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_LYING_ON_SURFACE) {
                Log.i(TAG, status + "Device lying on surface");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS) {
                Log.i(TAG, status + "Too few depth points");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES) {
                Log.i(TAG, status + "Too few features");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID) {
                Log.i(TAG, status + "Invalid poses in MotionTracking");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST) {
                Log.i(TAG, status + "Moving too fast");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_OVER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Over Exposed");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_UNDER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Under Exposed");
            }
        }
    };

    /**
     * First Person button onClick callback.
     */
    public void onFirstPersonClicked(View v) {
        mRenderer.setFirstPersonView();
    }

    /**
     * Third Person button onClick callback.
     */
    public void onThirdPersonClicked(View v) {
        mRenderer.setThirdPersonView();
    }

    /**
     * Top-down button onClick callback.
     */
    public void onTopDownClicked(View v) {
        mRenderer.setTopDownView();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mRenderer.onTouchEvent(event);
        return true;
    }

    /**
     * Calculates the average depth from a point cloud buffer.
     *
     * @param pointCloudBuffer
     * @param numPoints
     * @return Average depth.
     */
    private float getAveragedDepth(FloatBuffer pointCloudBuffer, int numPoints) {
        float totalZ = 0;
        float averageZ = 0;
        if (numPoints != 0) {
            int numFloats = 4 * numPoints;
            for (int i = 2; i < numFloats; i = i + 4) {
                totalZ = totalZ + pointCloudBuffer.get(i);
            }
            averageZ = totalZ / numPoints;
        }
        return averageZ;
    }

    /**
     * Query the display's rotation.
     */
    private void setDisplayRotation() {
        Display display = getWindowManager().getDefaultDisplay();
        mDisplayRotation = display.getRotation();
    }

    /**
     * Display toast on UI thread.
     *
     * @param resId The resource id of the string resource to use. Can be formatted text.
     */
    private void showsToastAndFinishOnUiThread(final int resId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(PointCloudActivity.this,
                        getString(resId), Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }

    private void writeToFile(String data, Context context) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("examplePointCloud.txt", Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        } catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }


    private DeviceExtrinsics setupExtrinsics(Tango mTango) {
        //camera to IMU tranform
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
        TangoPoseData imu_T_rgb = mTango.getPoseAtTime(0.0, framePair);
        //IMU to device transform
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        TangoPoseData imu_T_device = mTango.getPoseAtTime(0.0, framePair);
        //IMU to depth transform
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH;
        TangoPoseData imu_T_depth = mTango.getPoseAtTime(0.0, framePair);
        return new DeviceExtrinsics(imu_T_device, imu_T_rgb, imu_T_depth);
    }

    public ArrayList<Vector3> normalize(TangoXyzIjData cloud, TangoPoseData cameraPose, DeviceExtrinsics extrinsics) {
        ArrayList<Vector3> normalizedCloud = new ArrayList<>();

        TangoPoseData camera_T_imu = ScenePoseCalculator.matrixToTangoPose(extrinsics.getDeviceTDepthCamera());

        while (cloud.xyz.hasRemaining()) {
            Vector3 rotatedV = ScenePoseCalculator.getPointInEngineFrame(
                    new Vector3(cloud.xyz.get(), cloud.xyz.get(), cloud.xyz.get()),
                    camera_T_imu,
                    cameraPose
            );
            normalizedCloud.add(rotatedV);
        }

        return normalizedCloud;
    }


    private void savePointCloud(final TangoXyzIjData xyzIj) {
        File directoryName = getAlbumStorageDir("test.txt");
        FileOutputStream out = null;
        try {
            out = new FileOutputStream(directoryName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        byte[] buffer = new byte[xyzIj.xyzCount * 3 * 4];
        FileInputStream fileStream = new FileInputStream(
                xyzIj.xyzParcelFileDescriptor.getFileDescriptor());
        int read;
        try {
            while ((read = fileStream.read(buffer)) != 1) {
                try {
                    if (out != null) {
                        out.write(buffer, 0, read);
                        out.close();
                        System.out.println("Printed to file");
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public File getAlbumStorageDir(String dirName) {
        if (!isExternalStorageWritable()) {
            return null;
        } else {
            // Get the directory for the user's public downloads directory.
            File file = new File(Environment.getExternalStoragePublicDirectory(
                    Environment.DIRECTORY_DOWNLOADS), dirName);
            if (!file.mkdirs() || !file.exists()) {
                Log.e(TAG, "Directory not created");
                return null;
            }
            return file;
        }
    }

    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if ((Environment.MEDIA_MOUNTED.equals(state)
                && Environment.MEDIA_MOUNTED_READ_ONLY.equals(state))) {
            return true;
        } else {
            Log.e(TAG, "External storage is not mounted READ/WRITE.");
            return false;
        }
    }

}
