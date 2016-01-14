package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceView;
import android.view.View;
import android.widget.LinearLayout;

import com.qualcomm.ftcrobotcontroller.R;
import com.example.rmmurphy.alotovisionlib.android.Cameras;
import com.example.rmmurphy.alotovisionlib.robotVision.RobotVision;
import com.example.rmmurphy.alotovisionlib.util.FPS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

/**
 * Core OpMode class containing most OpenCV functionality
 */
abstract class VisionOpModeCore extends OpMode implements View.OnTouchListener, CameraBridgeViewBase.CvCameraViewListener2 {
    private static final int initialMaxSize = 1200;
    private static CameraBridgeViewBase openCVCamera;
    private static boolean initialized = false;
    private static boolean openCVInitialized = false;
    private boolean waitFirstTouch = true;
    private int firstTouchX = 0;
    private int firstTouchY = 0;

    private final BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    //Woohoo!
                    Log.d("OpenCV", "OpenCV Manager connected!");
                    openCVInitialized = true;
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };
    protected int width, height;
    protected FPS fps;
    protected LinearLayout entireScreenLayout;
    private CameraBridgeViewBase mOpenCvCameraView;
    public RobotVision rbVis;
    public Mat mSpectrum;
    public Size SPECTRUM_SIZE;
    public Scalar CONTOUR_COLOR;

    public final void setCamera(Cameras camera) {
        mOpenCvCameraView.disconnectCamera();
        mOpenCvCameraView.setCameraIndex(camera.getID());
        mOpenCvCameraView.connectCamera(width, height);


    }

    public final void setFrameSize(Size frameSize) {
        mOpenCvCameraView.setMaxFrameSize((int) frameSize.width, (int) frameSize.height);

        mOpenCvCameraView.disconnectCamera();
        mOpenCvCameraView.connectCamera((int) frameSize.width, (int) frameSize.height);

        width = mOpenCvCameraView.getFrameWidth();
        height = mOpenCvCameraView.getFrameHeight();
    }

    @Override
    public void init() {
        //Initialize camera view
        final Activity activity = (Activity) hardwareMap.appContext;
        final VisionOpModeCore t = this;

        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            boolean success = OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, mLoaderCallback);
            if (!success)
                Log.e("OpenCV", "Asynchronous initialization failed!");
            else
                Log.d("OpenCV", "Asynchronous initialization succeeded!");
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        while (!openCVInitialized) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {

                //Initialize FPS counter
                fps = new FPS();

                mSpectrum = new Mat();
                SPECTRUM_SIZE = new Size(100, 35);
                CONTOUR_COLOR = new Scalar(0, 255, 0, 255);

                mOpenCvCameraView = (CameraBridgeViewBase) activity.findViewById(R.id.robot_vision_view);
                mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
                mOpenCvCameraView.setCvCameraViewListener(t);
                //mOpenCvCameraView.connectCamera(initialMaxSize, initialMaxSize);
                mOpenCvCameraView.setOnTouchListener(t);
                mOpenCvCameraView.enableView();

                //Done!
                width = mOpenCvCameraView.getFrameWidth();
                height = mOpenCvCameraView.getFrameHeight();

                rbVis = new RobotVision(width, height);
                initialized = true;
                waitFirstTouch = true;
            }
        });

        while (!initialized) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void loop() {

    }

    abstract Mat frame(CameraBridgeViewBase.CvCameraViewFrame inputFrame, Mat rgba, Mat gray);

    @Override
    public void stop() {
        super.stop();

        if (mOpenCvCameraView != null){
            mOpenCvCameraView.disableView();
        }

    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        this.width = width;
        this.height = height;
    }

    @Override
    public void onCameraViewStopped() {

    }

    public boolean onTouch(View v, MotionEvent event)
    {
        int cols = rbVis.getCameraImage().rgba().cols();
        int rows = rbVis.getCameraImage().rgba().rows();

        rbVis.setObjectTrackState(RobotVision.State.OBJECT_IDLE);

        if( rbVis.getObjectTrackState() == RobotVision.State.OBJECT_IDLE )
        {
            int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
            int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;

            int x = (int) event.getX() - xOffset;
            int y = (int) event.getY() - yOffset;

            if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

            Rect touchedRect = new Rect();

            touchedRect.x = (x > 8) ? x - 8 : 0;
            touchedRect.y = (y > 8) ? y - 8 : 0;

            touchedRect.width = (x + 8 < cols) ? x + 8 - touchedRect.x : cols - touchedRect.x;
            touchedRect.height = (y + 8 < rows) ? y + 8 - touchedRect.y : rows - touchedRect.y;

            rbVis.setObjectTrackInitRect(touchedRect);
            rbVis.setObjectTrackState(RobotVision.State.OBJECT_TRACK_INIT);
        }
        return false; // don't need subsequent touch events
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        if (!initialized) {
            return inputFrame.rgba();
        }

        telemetry.addData("Vision Status", "Ready!");

        fps.update();
        return frame(inputFrame, inputFrame.rgba(), inputFrame.gray());
    }

}
