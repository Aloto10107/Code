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
    public List<List<Double>> blobs;

    public RobotVision()
    {
        blobs = new ArrayList<List<Double>>();
    }

    public void setCameraImage(CvCameraViewFrame currentImage)
    {
        this.currentImage=currentImage;
    }

    public CvCameraViewFrame getCameraImage()
    {
        return this.currentImage;
    }

    public void setBlobs(List<List<Double>> blobs)
    {
        this.blobs=blobs;
    }

    public List<List<Double>> getBlobs()
    {
        return this.blobs;
    }

    public Scalar getBlobColor( List<Double> blob)
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

        Rect blobRect = new Rect(x,y,width,height);

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

    public List<List<Double>> findBlobs( ColorBlobDetector mDetector, CvCameraViewFrame currentImage)
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

            x = rect.x + rect.width/2;
            y = rect.y + rect.height/2;
            area = rect.area();
            //Center coordinate x
            singleAddress.add((double)(x - cols / 2));
            //Rect starting coordinate x
            singleAddress.add((double)rect.x);
            //Center coordinate y
            singleAddress.add((double)(-y + rows / 2));
            //Rect starting coordinate y
            singleAddress.add((double)rect.y);
            //Rect width
            singleAddress.add((double)rect.width);
            //Rect height
            singleAddress.add((double)rect.height);
            //Blob area
            singleAddress.add(area);

            addresses.add(singleAddress);
        }

        this.blobs        = addresses;
        this.currentImage = currentImage;
        return this.blobs;
    }

}
