package com.qualcomm.ftcrobotcontroller;

import android.app.Application;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
   private State objectTrackState;
   private Scalar objectColorHsv;
   private Scalar objectColorRgb;
   private Rect objectTrackingRect;
   private Rect objectTrackInitRect;

   public enum State
   {
      OBJECT_TRACK_INIT, OBJECT_TRACK, OBJECT_LOST, OBJECT_IDLE
   }

   public List<List<Double>> blobs;

   public RobotVision()
   {
      this.blobs = new ArrayList<List<Double>>();
      this.objectTrackState = State.OBJECT_IDLE;
      this.objectLostCount = 0;
      this.objectColorHsv = new Scalar(255);
      this.objectColorRgb = new Scalar(255);
      this.objectTrackingRect = new Rect();
      this.objectTrackInitRect = new Rect();
   }/*End RobotVision*/

   public Scalar getObjectColorRgb()
   {
      return this.objectColorRgb;
   }

   public Scalar getObjectColorHsv()
   {
      return this.objectColorHsv;
   }

   public void setObjectTrackInitRect(Rect trackRect)
   {
      this.objectTrackInitRect = trackRect;
   }

   public Rect getObjectTrackInitRect()
   {
      return this.objectTrackInitRect;
   }

   public Rect getObjectTrackingRect()
   {
      return this.objectTrackingRect;
   }

   public void setObjectTrackState(State objectState)
   {
      this.objectTrackState = objectState;
   }

   public State getObjectTrackState()
   {
      return this.objectTrackState;
   }

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

   private Scalar convertScalarHsv2Rgba(Scalar hsvColor)
   {
      Mat pointMatRgba = new Mat();
      Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
      Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

      return new Scalar(pointMatRgba.get(0, 0));
   }

   public ColorBlobDetector updateObjectTrack(ColorBlobDetector mDetector,
                                 CvCameraViewFrame currentImage,
                                 Mat mSpectrum,
                                 Size SPECTRUM_SIZE)
   {
       /*-------------------------------------------------------------------------------------------
        * Store the current camera image...
        *-----------------------------------------------------------------------------------------*/
      this.currentImage = currentImage;
      int cols = currentImage.rgba().cols();
      int rows = currentImage.rgba().rows();

      switch(this.objectTrackState)
      {
         case OBJECT_TRACK_INIT:
         {
              /*------------------------------------------------------------------------------------
               * Find the average bounding rectangle and average color within the bounds for the
               * desired object to track. Find the center coordinates of the bounding rectangle and
               * initialize a 4 state Kalman filter to track the object.
               *----------------------------------------------------------------------------------*/
            Mat touchedRegionRgba = currentImage.rgba().submat(this.objectTrackInitRect);

            Mat touchedRegionHsv = new Mat();
            Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

            // Calculate average color of touched region
            this.objectColorHsv = Core.sumElems(touchedRegionHsv);
            int pointCount = this.objectTrackInitRect.width * this.objectTrackInitRect.height;
            for(int i = 0; i < this.objectColorHsv.val.length; i++)
               this.objectColorHsv.val[i] /= pointCount;

            //this.objectColorHsv.val[0] = 120;
            //this.objectColorHsv.val[1] = 255;
            //this.objectColorHsv.val[2] = 255;

            this.objectColorRgb = convertScalarHsv2Rgba(this.objectColorHsv);

            mDetector.setHsvColor(this.objectColorHsv);

            //Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

            touchedRegionRgba.release();
            touchedRegionHsv.release();

            /*--------------------------------------------------------------------------------------
             * Find the blob in the region that was touched.
             *------------------------------------------------------------------------------------*/
            List<List<Double>> blobs = this.findBlobs(mDetector, currentImage, false);

            Double area;
            int x;
            int y;
            int width;
            int height;
            boolean objectMatch = false;
            int blobNumber = 0;

            for(int i = 0; i < blobs.size(); i++)
            {
               //Scalar blobColor = g.getBlobColor(blobs.get(i));
               x = blobs.get(i).get(1).intValue();
               y = blobs.get(i).get(3).intValue();
               width = blobs.get(i).get(4).intValue();
               height = blobs.get(i).get(5).intValue();

               /*-----------------------------------------------------------------------------------
                * If touched rect is inside the blob rect then its a match.
                *---------------------------------------------------------------------------------*/
               int centerx = this.objectTrackInitRect.x + this.objectTrackInitRect.width/2;
               int centery = this.objectTrackInitRect.y + this.objectTrackInitRect.height/2;
               boolean test1 = (centerx >= x) && (centerx <= (x + width));
               boolean test2 = (centery >= y) && (centery <= (y + height));
               if(test1 && test2)
               {
                  objectMatch = true;
                  blobNumber = i;
                  break;
               }
            }

            this.objectLostCount = 0;
            if(objectMatch)
            {
               /*-----------------------------------------------------------------------------------
                * Object is a match, set the tracking rect to a size 1.5 times larger that the
                * object rectangle
                *---------------------------------------------------------------------------------*/
               x = blobs.get(blobNumber).get(1).intValue();
               y = blobs.get(blobNumber).get(3).intValue();
               width = blobs.get(blobNumber).get(4).intValue();
               height = blobs.get(blobNumber).get(5).intValue();

               if( (x - width / 2) < 0)
                  this.objectTrackingRect.x = 0;
               else
                  this.objectTrackingRect.x = x - width / 2;

               if( (y - height / 2) < 0)
                  this.objectTrackingRect.y = 0;
               else
                  this.objectTrackingRect.y = y - height / 2;

               if( (this.objectTrackingRect.x + width * 2) > cols)
                  this.objectTrackingRect.width = width * 2 - (this.objectTrackingRect.x + width * 2) - cols;
               else
                  this.objectTrackingRect.width = width * 2;

               if( (this.objectTrackingRect.y + height * 2) > rows)
                  this.objectTrackingRect.height = height * 2 - (this.objectTrackingRect.y + height * 2) - rows;
               else
                  this.objectTrackingRect.height = height * 2;

               this.objectTrackState = State.OBJECT_TRACK;
            }
            else
               this.objectTrackState = State.OBJECT_IDLE;

            break;/*End case OBJECT_TRACK_INIT:*/
         }
         case OBJECT_TRACK:
         {
              /*------------------------------------------------------------------------------------
               * Track the location of the object using a Kalman filter. Use the qualtiy of the
               * covariance matrix to adjust the size of the "tracking" bounding rectangle i.e. i
               * f the covariance is large make the bounding rectangle large, otherwise make it
               * small. If the blob detection algorithm hasn't identified the object for a preset
               * period of time, go to the object lost state.
               *----------------------------------------------------------------------------------*/
            List<List<Double>> blobs = this.findBlobs(mDetector, currentImage, true);

            if(this.objectLostCount == OBJECT_TRACK_TIMEOUT)
            {
               this.objectTrackState = State.OBJECT_LOST;
            }/*End if( this.objectLostCount == OBJECT_TRACK_TIMEOUT)*/

            break;/*End case OBJECT_TRACK:*/
         }
         case OBJECT_LOST:
            this.objectLostCount = 0;
            break;/*End case OBJECT_LOST:*/

         case OBJECT_IDLE:
              /*------------------------------------------------------------------------------------
               * This state is entered after boot, waiting for user to touch an object.
               *----------------------------------------------------------------------------------*/
            break;/*End case OBJECT_IDLE:*/

      }/*End switch( this.objectTrackState)*/

       /*-------------------------------------------------------------------------------------------
        * Return the center coordinates, "tracking" bounding rectangle, and average HSV color of the
        * tracked object, use this information to scale the image such that the blob detection
        * algorithm looks in a specific area for the object of interest.
        *-----------------------------------------------------------------------------------------*/
      return mDetector;
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
      for(int i = 0; i < mBlobColorHsv.val.length; i++)
         mBlobColorHsv.val[i] /= pointCount;

      blobRegionRgba.release();
      blobRegionHsv.release();

      return mBlobColorHsv;
   }

   public List<List<Double>> findBlobs(ColorBlobDetector mDetector, CvCameraViewFrame currentImage, boolean useSubMat)
   {
      int cols = currentImage.rgba().cols();
      int rows = currentImage.rgba().rows();

      if( useSubMat)
      {

         Mat trackedRgba = currentImage.rgba().submat(this.objectTrackingRect);
         mDetector.process(trackedRgba);
         //cols = trackedRgba.cols();
         //rows = trackedRgba.rows();
      }
      else
         mDetector.process(currentImage.rgba());

      List<MatOfPoint> contours = mDetector.getContours();

      MatOfPoint2f approxCurve = new MatOfPoint2f();
      Double area;
      int x;
      int y;
      int width;
      int height;
      List<List<Double>> addresses = new ArrayList<List<Double>>();

      for(int i = 0; i < contours.size(); i++)
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

         if( useSubMat)
         {
            rect.x = rect.x + this.objectTrackingRect.x;
            rect.y = rect.y + this.objectTrackingRect.y;
         }

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
