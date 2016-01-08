package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.VisionOpModeCore;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

public abstract class ManualVisionOpMode extends VisionOpModeCore {
    public final Mat frame(CameraBridgeViewBase.CvCameraViewFrame inputFrame, Mat rgba, Mat gray, boolean ready) {
        return frame(inputFrame, rgba, gray);
    }

    public abstract Mat frame(CameraBridgeViewBase.CvCameraViewFrame inputFrame, Mat rgba, Mat gray);
}
