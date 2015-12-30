package com.qualcomm.ftcrobotcontroller;

import android.app.Application;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.boundingRect;

/**
 * Created by rmmurphy on 12/29/2015.
 */
public class RobotVision extends Application
{
    private CvCameraViewFrame currentImage;
    private int objectLostCount;
    private static int OBJECT_TRACK_TIMEOUT = 10;

    public enum State
    {
        OBJECT_TRACK_INIT, OBJECT_TRACK, OBJECT_LOST
    }

    public State robotState;

    public List<List<Double>> blobs;

    public RobotVision()
    {
        this.blobs           = new ArrayList<List<Double>>();
        this.robotState      = State.OBJECT_TRACK_INIT;
        this.objectLostCount = 0;
    }/*End RobotVision*/

    public void setCameraImage(CvCameraViewFrame currentImage)
    {
        this.currentImage = currentImage;
    }/*End setCameraImage*/

    public CvCameraViewFrame getCameraImage()
    {
        return this.currentImage;
    }

    public void setBlobs(List<List<Double>> blobs)
    {
        this.blobs = blobs;
    }

    public List<List<Double>> getBlobs()
    {
        return this.blobs;
    }/*End getBlobs*/

    public void updateObjectTrack( ColorBlobDetector mDetector,
                                   CvCameraViewFrame currentImage,
                                   boolean objectTouched)
    {

       /*-------------------------------------------------------------------------------------------
        * When the screen is pressed, go to the target initialization state.
        *-----------------------------------------------------------------------------------------*/
       if( objectTouched)
          this.robotState = State.OBJECT_TRACK_INIT;

        switch( this.robotState)
        {
           case OBJECT_TRACK_INIT:
              /*------------------------------------------------------------------------------------
               * Find the average bounding rectangle and average color within the bounds for the
               * desired object to track. Find the center coordinates of the bounding rectangle and
               * initialize a 4 state Kalman filter to track the object.
               *----------------------------------------------------------------------------------*/
              this.objectLostCount = 0;
              this.robotState = State.OBJECT_TRACK;
              break;/*End case OBJECT_TRACK_INIT:*/

           case OBJECT_TRACK:
              /*------------------------------------------------------------------------------------
               * Track the location of the object using a Kalman filter. Use the qualtiy of the
               * covariance matrix to adjust the size of the "tracking" bounding rectangle i.e. i
               * f the covariance is large make the bounding rectangle large, otherwise make it
               * small. If the blob detection algorithm hasn't identified the object for a preset
               * period of time, go to the object lost state.
               *----------------------------------------------------------------------------------*/
              if( this.objectLostCount == OBJECT_TRACK_TIMEOUT)
              {
                 this.robotState = State.OBJECT_LOST;
              }/*End if( this.objectLostCount == OBJECT_TRACK_TIMEOUT)*/

              break;/*End case OBJECT_TRACK:*/

           case OBJECT_LOST:
              this.objectLostCount = 0;
              break;/*End case OBJECT_LOST:*/

        }/*End switch( this.robotState)*/

       /*-------------------------------------------------------------------------------------------
        * Return the center coordinates, "tracking" bounding rectangle, and average HSV color of the
        * tracked object, use this information to scale the image such that the blob detection
        * algorithm looks in a specific area for the object of interest.
        *-----------------------------------------------------------------------------------------*/
    }/*End updateObjectTrack*/

    public Scalar getBlobColor(List<Double> blob)
    {
        int x;
        int y;
        int width;
        int height;

        Scalar mBlobColorHsv = new Scalar(255);

        x = blob.get(1).intValue();
        y = blob.get(3).intValue();
        width = blob.get(4).intValue();
        height = blob.get(5).intValue();

        Rect blobRect = new Rect(x, y, width, height);

        Mat blobRegionRgba = currentImage.rgba().submat(blobRect);

        Mat blobRegionHsv = new Mat();
        Imgproc.cvtColor(blobRegionRgba, blobRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        mBlobColorHsv = Core.sumElems(blobRegionHsv);
        int pointCount = blobRect.width * blobRect.height;
        for (int i = 0; i < mBlobColorHsv.val.length; i++)
            mBlobColorHsv.val[i] /= pointCount;

        return mBlobColorHsv;
    }

    public void setObjectToTrack(ColorBlobDetector mDetector, CvCameraViewFrame currentImage)
    {

    }

    public List<List<Double>> findBlobs(ColorBlobDetector mDetector, CvCameraViewFrame currentImage)
    {
        int cols = currentImage.rgba().cols();
        int rows = currentImage.rgba().rows();

        mDetector.process(currentImage.rgba());
        List<MatOfPoint> contours = mDetector.getContours();

        MatOfPoint2f approxCurve = new MatOfPoint2f();
        Double area;
        int x;
        int y;
        int width;
        int height;
        List<List<Double>> addresses = new ArrayList<List<Double>>();

        for (int i = 0; i < contours.size(); i++)
        {
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

            //Convert back to MatOfPoint
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());

            // Get bounding rect of contour
            Rect rect = boundingRect(points);

            ArrayList<Double> singleAddress = new ArrayList<Double>();

            x = rect.x + rect.width / 2;
            y = rect.y + rect.height / 2;
            area = rect.area();
            //Center coordinate x
            singleAddress.add((double) (x - cols / 2));
            //Rect starting coordinate x
            singleAddress.add((double) rect.x);
            //Center coordinate y
            singleAddress.add((double) (-y + rows / 2));
            //Rect starting coordinate y
            singleAddress.add((double) rect.y);
            //Rect width
            singleAddress.add((double) rect.width);
            //Rect height
            singleAddress.add((double) rect.height);
            //Blob area
            singleAddress.add(area);

            addresses.add(singleAddress);
        }

        this.blobs = addresses;
        this.currentImage = currentImage;
        return this.blobs;
    }

}
