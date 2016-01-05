package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.robotVision.RobotVision;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.List;

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
    public Mat frame(CameraBridgeViewBase.CvCameraViewFrame inputFrame, Mat rgba, Mat gray) {

      /*--------------------------------------------------------------------------------------------
       * Track the color, coordinates, and area of the selected object.
       *------------------------------------------------------------------------------------------*/
        rbVis.updateObjectTrack( inputFrame);

        if(rbVis.getObjectTrackState() == RobotVision.State.OBJECT_TRACK)
        {

            Imgproc.resize(rbVis.getBlobDetector().getSpectrum(), mSpectrum, SPECTRUM_SIZE);
            List<List<Double>> blobs = rbVis.getBlobs();

            Double area;
            int x;
            int y;
            int width;
            int height;

            //Plot the blob locations
            for(int i = 0; i < blobs.size(); i++)
            {
                //Scalar blobColor = rbVis.getBlobColor(blobs.get(i));
                x = blobs.get(i).get(1).intValue();
                y = blobs.get(i).get(3).intValue();
                width = blobs.get(i).get(4).intValue();
                height = blobs.get(i).get(5).intValue();
                Imgproc.rectangle(rgba, new Point(x, y), new Point(x + width, y + height), new Scalar(0, 255, 0, 255), 3);
                x = x + width / 2;
                y = y + height / 2;
                area = blobs.get(i).get(6);

                Imgproc.putText(rgba, "[" + blobs.get(i).get(0).intValue() + "," + blobs.get(i).get(2).intValue() + "," + area.intValue() + "]", new Point(x + 4, y), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 255, 255), 3);
                Imgproc.circle(rgba, new Point(x, y), 5, new Scalar(0, 255, 0, 255), -1);
            }

            Rect rec = rbVis.getObjectTrackingRect();

            Imgproc.rectangle(rgba, new Point(rec.x, rec.y), new Point(rec.x + rec.width, rec.y + rec.height), new Scalar(0, 0, 255, 255), 3);

            Mat colorLabel = rgba.submat(4, 40, 4, 40);
            colorLabel.setTo(rbVis.getObjectColorRgb());

            Mat spectrumLabel = rgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);

        }/*End if( rbVis.getObjectTrackState() == RobotVision.State.OBJECT_TRACK)*/
        
        return rgba;
    }

    @Override
    public void stop() {
        super.stop();

    }
}
