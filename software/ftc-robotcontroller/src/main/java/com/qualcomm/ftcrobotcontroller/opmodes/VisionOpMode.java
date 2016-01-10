package com.qualcomm.ftcrobotcontroller.opmodes;

import com.example.rmmurphy.alotovisionlib.robotVision.RobotVision;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

/**
 * Easy-to-use, extensible vision op mode
 * For more custom implementations, use ManualVisionOpMode or modify core extensions in opmode.extensions.*
 */
public abstract class VisionOpMode extends VisionOpModeCore {

    /***
     * EXTENSION-SPECIFIC CODE
     ***/
    private boolean initialized = false;


    @Override
    public void init() {
        super.init();

        initialized = true;
    }

    @Override
    public void loop() {
        super.loop();

    }

    @Override
    public Mat frame(CameraBridgeViewBase.CvCameraViewFrame inputFrame, Mat rgba, Mat gray)
    {

        int cols = rgba.cols();
        int rows = rgba.rows();

      /*--------------------------------------------------------------------------------------------
       * Track the color, coordinates, and area of the selected object.
       *------------------------------------------------------------------------------------------*/
        rbVis.updateObjectTrack(inputFrame);

        if (rbVis.isTargetLocked())
        {
            Imgproc.resize(rbVis.getBlobDetector().getSpectrum(), mSpectrum, SPECTRUM_SIZE);

            Double area;
            int x;
            int y;
            int width;
            int height;

            int[] rawTarget = rbVis.getRawTargetCoords();
            double[] filteredTarget = rbVis.getFilteredTargetCoords();

            if( rawTarget != null)
            {
                /*---------------------------------------------------------------------------------*
                 * Draw the raw target bounding rect.
                 *--------------------------------------------------------------------------------*/
                x = rawTarget[0] - (rawTarget[2] / 2);
                y = rawTarget[1] - (rawTarget[3] / 2);

                Imgproc.rectangle(rgba, new Point(x, y), new Point(x + rawTarget[2], y + rawTarget[3]), new Scalar(0, 255, 0, 255), 3);

                //Imgproc.circle(mRgba, new Point(rawTarget[0], rawTarget[1]), 5, new Scalar(0, 255, 0, 255), -1);

                Rect rec = rbVis.getRegionOfInterestRect();
                Imgproc.rectangle(rgba, new Point(rec.x, rec.y), new Point(rec.x + rec.width, rec.y + rec.height), new Scalar(0, 0, 255, 255), 3);
            }
            Imgproc.putText(rgba, "[" + (int)filteredTarget[0] + "," + (int)filteredTarget[1] + "," + (int)(filteredTarget[4] * filteredTarget[5]) + "]", new Point((int)filteredTarget[0]+cols/2 + 4, -(int)filteredTarget[1]+rows/2), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 255, 255), 3);
            Imgproc.circle(rgba, new Point((int)filteredTarget[0]+cols/2, -((int)filteredTarget[1]+rows/2)), 5, new Scalar(0, 255, 0, 255), -1);

            Mat colorLabel = rgba.submat(4, 40, 4, 40);
            colorLabel.setTo(rbVis.getObjectColorRgb());

            Mat spectrumLabel = rgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);

        }/*End if( g.getObjectTrackState() == RobotVision.State.OBJECT_TRACK)*/

        return rgba;
    }

    @Override
    public void stop() {
        super.stop();

    }
}
