package com.example.rmmurphy.alotovisionlib.robotVision;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.KalmanFilter;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.boundingRect;

/**
 * Created by rmmurphy on 12/29/2015.
 */
public class RobotVision
{
   private CvCameraViewFrame currentImage;
   private int objectLostCount;
   private static int OBJECT_TRACK_TIMEOUT = 100;
   private static int TRACK_RECT_MAX_SCALAR = 3;
   private static double HEIGHT_WIDTH_FILTER_AMOUNT = 10;
   private State objectTrackState;
   private State currentObjectTrackState;
   private Scalar objectColorHsv;
   private Scalar objectColorRgb;
   private Rect regionOfInterestRect;
   private Rect initialRegionOfInterestRect;
   private Rect rawTarget;
   private long prevNanoTime;
   private ColorBlobDetector mDetector;
   private KalmanFilter kalman;
   private Mat kalmanTransMatrix;
   private Mat kalmanProcNoiseCov;
   private Mat kalmanMeasNoiseCov;
   private Mat kalmanMeas;
   private int frameRows;
   private int frameCols;
   private Mat kalmanTrackedState;
   private double[] filteredTargetWidthHeight;

   public enum State
   {
      OBJECT_TRACK_INIT, OBJECT_TRACK, OBJECT_LOST, OBJECT_IDLE
   }

   public ArrayList<Rect> blobs;

   public RobotVision( int width, int height)
   {
      /*--------------------------------------------------------------------------------------------
       * Declare a 4 state kalman filter where state 0 = x, state 1 = y, state 2 = x', and state 3=
       * y'.
       *------------------------------------------------------------------------------------------*/

      kalman             = new KalmanFilter(4, 2, 0, CvType.CV_32F);
      kalmanTransMatrix  = new Mat(4, 4, CvType.CV_32F, new Scalar(0));
      kalmanProcNoiseCov = new Mat(4, 4, CvType.CV_32F, new Scalar(0));
      kalmanMeasNoiseCov = new Mat(2, 2, CvType.CV_32F, new Scalar(0));
      kalmanMeas         = new Mat(2, 1, CvType.CV_32F, new Scalar(0));

      /*--------------------------------------------------------------------------------------------
       * Initialize the state transition matrix...dt isn't known yet so set the value later.
       *                     | 1  0  dt  0  |
       * kalmanTransMatrix = | 0  1   0  dt |
       *                     | 0  0   1  0  |
       *                     | 0  0   0  1  |
       *
       *------------------------------------------------------------------------------------------*/
      kalmanTransMatrix.put(0,0, 1.0);
      kalmanTransMatrix.put(0,1, 0.0);
      kalmanTransMatrix.put(0,2, 1.0); //dt place holder
      kalmanTransMatrix.put(0,3, 0.0);

      kalmanTransMatrix.put(1,0, 0.0);
      kalmanTransMatrix.put(1,1, 1.0);
      kalmanTransMatrix.put(1,2, 0.0);
      kalmanTransMatrix.put(1,3, 1.0); //dt place holder

      kalmanTransMatrix.put(2,0, 0.0);
      kalmanTransMatrix.put(2,1, 0.0);
      kalmanTransMatrix.put(2,2, 1.0);
      kalmanTransMatrix.put(2,3, 0.0);

      kalmanTransMatrix.put(3,0, 0.0);
      kalmanTransMatrix.put(3,1, 0.0);
      kalmanTransMatrix.put(3,2, 0.0);
      kalmanTransMatrix.put(3,3, 1.0);

      kalman.set_transitionMatrix(kalmanTransMatrix);

      /*--------------------------------------------------------------------------------------------
       * Kalman filter H matrix
       *------------------------------------------------------------------------------------------*/
      Mat meas         = kalman.get_measurementMatrix();
      Mat procNoiseCov = kalman.get_processNoiseCov();
      Mat measNoiseCov = kalman.get_measurementNoiseCov();
      Mat errorCovPost = kalman.get_errorCovPost();

      /*--------------------------------------------------------------------------------------------
       * Observing the measurements directly so the H matrix is the identity.
       *------------------------------------------------------------------------------------------*/
      Core.setIdentity( meas);
      Core.setIdentity(kalmanProcNoiseCov, Scalar.all(1e-2));
      Core.setIdentity(kalmanMeasNoiseCov, Scalar.all(1e-4));

      /*--------------------------------------------------------------------------------------------
       * We have known starting coordinates so set this value high.
       *------------------------------------------------------------------------------------------*/
      Core.setIdentity( errorCovPost, Scalar.all(1));

      kalman.set_measurementMatrix(meas);
      kalman.set_processNoiseCov(kalmanProcNoiseCov);
      kalman.set_measurementNoiseCov(kalmanMeasNoiseCov);
      kalman.set_errorCovPost(errorCovPost);

      mDetector = new ColorBlobDetector();

      this.blobs = new ArrayList<Rect>();
      this.objectTrackState = State.OBJECT_IDLE;
      this.currentObjectTrackState = State.OBJECT_IDLE;
      this.objectLostCount = 0;
      this.objectColorHsv = new Scalar(255);
      this.objectColorRgb = new Scalar(255);
      this.regionOfInterestRect = new Rect();
      this.initialRegionOfInterestRect = new Rect();
      this.frameRows = height;
      this.frameCols = width;
      this.kalmanTrackedState = new Mat(4, 1, CvType.CV_32F, new Scalar(0));
      this.rawTarget = new Rect();
      filteredTargetWidthHeight = new double[4];
   }

   public ColorBlobDetector getBlobDetector()
   {
      return this.mDetector;
   }

   public KalmanFilter getKalmanFilter()
   {
      return this.kalman;
   }

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

      this.initialRegionOfInterestRect = trackRect;
   }

   public Rect getObjectTrackInitRect()
   {
      return this.initialRegionOfInterestRect;
   }

   public Rect getRegionOfInterestRect()
   {
      int cols = frameCols;
      int rows = frameRows;
      Rect newRet = new Rect();

      newRet = regionOfInterestRect;

      if( newRet.x < 0)
         newRet.x = 0;

      if( newRet.x + newRet.width > cols)
         newRet.width = cols - newRet.x;

      if( newRet.y < 0)
         newRet.y = 0;

      if( newRet.y + newRet.height > rows)
         newRet.height = rows - newRet.y;

      return newRet;
   }

   public void setObjectTrackState(State objectState)
   {
      this.objectTrackState = objectState;
   }

   public State getObjectTrackState()
   {
      return this.currentObjectTrackState;
   }

   public boolean isTargetLocked()
   {
      if( currentObjectTrackState == State.OBJECT_TRACK)
         return true;
      else
         return false;
   }
   public void setCameraImage(CvCameraViewFrame currentImage)
   {
      this.currentImage = currentImage;
   }/*End setCameraImage*/

   public CvCameraViewFrame getCameraImage()
   {
      return this.currentImage;
   }

   private void setBlobs(ArrayList<Rect> blobs)
   {
      this.blobs = blobs;
   }

   private ArrayList<Rect> getBlobs()
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

   public void updateObjectTrack( CvCameraViewFrame currentImage)
   {
       /*-------------------------------------------------------------------------------------------
        * Store the current camera image...
        *-----------------------------------------------------------------------------------------*/
      this.currentImage = currentImage;
      int cols = currentImage.rgba().cols();
      int rows = currentImage.rgba().rows();
      long currentNanoTime = System.nanoTime();
      double deltaTimeSec =  (double)(currentNanoTime - prevNanoTime)  / 1000000000.0;;

      switch(this.objectTrackState)
      {
         case OBJECT_TRACK_INIT:
         {
            this.currentObjectTrackState = State.OBJECT_TRACK_INIT;
              /*------------------------------------------------------------------------------------
               * Find the average bounding rectangle and average color within the bounds for the
               * desired object to track. Find the center coordinates of the bounding rectangle and
               * initialize a 4 state Kalman filter to track the object.
               *----------------------------------------------------------------------------------*/
            Mat touchedRegionRgba = currentImage.rgba().submat(this.initialRegionOfInterestRect);

            Mat touchedRegionHsv = new Mat();
            Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

            // Calculate average color of touched region
            this.objectColorHsv = Core.sumElems(touchedRegionHsv);
            int pointCount = this.initialRegionOfInterestRect.width * this.initialRegionOfInterestRect.height;
            for(int i = 0; i < this.objectColorHsv.val.length; i++)
               this.objectColorHsv.val[i] /= pointCount;

            //this.objectColorHsv.val[0] = 120;
            //this.objectColorHsv.val[1] = 255;
            //this.objectColorHsv.val[2] = 255;

            this.objectColorRgb = convertScalarHsv2Rgba(this.objectColorHsv);

            this.mDetector.setHsvColor(this.objectColorHsv);

            //Imgproc.resize(this.mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

            touchedRegionRgba.release();
            touchedRegionHsv.release();

            /*--------------------------------------------------------------------------------------
             * Find the blob in the region that was touched.
             *------------------------------------------------------------------------------------*/
            List<Rect> blobs = this.findBlobs(currentImage, false);

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
               x = blobs.get(i).x;
               y = blobs.get(i).y;
               width = blobs.get(i).width;
               height = blobs.get(i).height;

               /*-----------------------------------------------------------------------------------
                * If touched rect is inside the blob rect then its a match.
                *---------------------------------------------------------------------------------*/
               int centerx = this.initialRegionOfInterestRect.x + this.initialRegionOfInterestRect.width/2;
               int centery = this.initialRegionOfInterestRect.y + this.initialRegionOfInterestRect.height/2;
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
               x = blobs.get(blobNumber).x;
               y = blobs.get(blobNumber).y;
               width = blobs.get(blobNumber).width;
               height = blobs.get(blobNumber).height;

               filteredTargetWidthHeight[0] = width;
               filteredTargetWidthHeight[1] = height;

               this.regionOfInterestRect.x = (int)((double)x + ((double) width / 2)  - ((double)width*TRACK_RECT_MAX_SCALAR/2));
               this.regionOfInterestRect.y = (int)((double)y + ((double) height / 2) - ((double)height*TRACK_RECT_MAX_SCALAR/2));
               this.regionOfInterestRect.width = width * TRACK_RECT_MAX_SCALAR;
               this.regionOfInterestRect.height = height * TRACK_RECT_MAX_SCALAR;

               /*-----------------------------------------------------------------------------------
                * Initialize the Kalman filter to the center coordinates of the object.
                *---------------------------------------------------------------------------------*/
               Mat statePred = kalman.get_statePre();

               statePred.put(0, 0, (double) x + (double) width / 2);
               statePred.put(1, 0, (double) y + (double) height / 2);
               statePred.put(2, 0, (double)0.0f);
               statePred.put(3, 0, (double) 0.0f);
               kalmanTrackedState = statePred;
               kalman.set_statePre(statePred);

               /*-----------------------------------------------------------------------------------
                * Update transition matrix dt
                *---------------------------------------------------------------------------------*/
               kalmanTransMatrix.put(0, 2, deltaTimeSec);
               kalmanTransMatrix.put(1, 3, deltaTimeSec);
               kalman.set_transitionMatrix(kalmanTransMatrix);

               //double[][] trans = new double[4][4];

               //trans[0][0] = kalmanTransMatrix.get(0,0)[0];
               //trans[0][1] = kalmanTransMatrix.get(0,1)[0];
               //trans[0][2] = kalmanTransMatrix.get(0,2)[0];
               //trans[0][3] = kalmanTransMatrix.get(0,3)[0];

               //trans[1][0] = kalmanTransMatrix.get(1,0)[0];
               //trans[1][1] = kalmanTransMatrix.get(1,1)[0];
               //trans[1][2] = kalmanTransMatrix.get(1,2)[0];
               //trans[1][3] = kalmanTransMatrix.get(1,3)[0];

               //trans[2][0] = kalmanTransMatrix.get(2,0)[0];
               //trans[2][1] = kalmanTransMatrix.get(2,1)[0];
               //trans[2][2] = kalmanTransMatrix.get(2,2)[0];
               //trans[2][3] = kalmanTransMatrix.get(2,3)[0];

               //trans[3][0] = kalmanTransMatrix.get(3,0)[0];
               //trans[3][1] = kalmanTransMatrix.get(3,1)[0];
               //trans[3][2] = kalmanTransMatrix.get(3,2)[0];
               //trans[3][3] = kalmanTransMatrix.get(3,3)[0];

               /*-----------------------------------------------------------------------------------
                * We have known starting coordinates so set this value high.
                *---------------------------------------------------------------------------------*/
               Mat errorCovPost = kalman.get_errorCovPost();
               Core.setIdentity(errorCovPost, Scalar.all(1));
               kalman.set_errorCovPost(errorCovPost);

               this.objectTrackState = State.OBJECT_TRACK;
            }
            else
               this.objectTrackState = State.OBJECT_IDLE;

            break;/*End case OBJECT_TRACK_INIT:*/
         }
         case OBJECT_TRACK:
         {
            this.currentObjectTrackState = State.OBJECT_TRACK;

            Rect target = findTarget();
            int x = (int)kalmanTrackedState.get(0,0)[0];
            int y = (int)kalmanTrackedState.get(1,0)[0];
            double dx = kalmanTrackedState.get(2,0)[0];
            double dy = kalmanTrackedState.get(3,0)[0];

              /*------------------------------------------------------------------------------------
               * Track the location of the object using a Kalman filter. Use the qualtiy of the
               * covariance matrix to adjust the size of the "tracking" bounding rectangle i.e. i
               * f the covariance is large make the bounding rectangle large, otherwise make it
               * small. If the blob detection algorithm hasn't identified the object for a preset
               * period of time, go to the object lost state.
               *----------------------------------------------------------------------------------*/

            /*--------------------------------------------------------------------------------------
             * Update transition matrix dt
             *------------------------------------------------------------------------------------*/
            kalmanTransMatrix.put(0,2, deltaTimeSec);
            kalmanTransMatrix.put(1,3, deltaTimeSec);
            kalman.set_transitionMatrix(kalmanTransMatrix);

            if( target != null)
            {
               /*-----------------------------------------------------------------------------------
                * Kalman measurement update...
                *---------------------------------------------------------------------------------*/
               kalmanMeas.put( 0, 0, (double)target.x + (double)target.width/2);
               kalmanMeas.put( 1, 0, (double)target.y + (double)target.height/2);
               kalmanTrackedState = kalman.correct( kalmanMeas);
               this.objectLostCount = 0;
               /*-----------------------------------------------------------------------------------
                * Move the object tracking rectangle based on the Kalman estimate coordinates.
                *---------------------------------------------------------------------------------*/
               x = (int)kalmanTrackedState.get(0,0)[0];
               y = (int)kalmanTrackedState.get(1,0)[0];
               dx = kalmanTrackedState.get(2,0)[0];
               dy = kalmanTrackedState.get(3,0)[0];

               filteredTargetWidthHeight[0] = filteredTargetWidthHeight[0]*( 1 - 1/HEIGHT_WIDTH_FILTER_AMOUNT) + (double)target.width*(1/HEIGHT_WIDTH_FILTER_AMOUNT);
               filteredTargetWidthHeight[1] = filteredTargetWidthHeight[1]*( 1 - 1/HEIGHT_WIDTH_FILTER_AMOUNT) + (double)target.height*(1/HEIGHT_WIDTH_FILTER_AMOUNT);

               regionOfInterestRect.x = x - ((int)filteredTargetWidthHeight[0]*TRACK_RECT_MAX_SCALAR)/2;
               regionOfInterestRect.y = y - ((int)filteredTargetWidthHeight[1]*TRACK_RECT_MAX_SCALAR)/2;

               regionOfInterestRect.width  = (int)filteredTargetWidthHeight[0]*TRACK_RECT_MAX_SCALAR;
               regionOfInterestRect.height = (int)filteredTargetWidthHeight[1]*TRACK_RECT_MAX_SCALAR;
            }
            else
            {
               /*-----------------------------------------------------------------------------------
                * Blob not detected use the predicted measurement as the estimate.
                *---------------------------------------------------------------------------------*/
               this.objectLostCount++;
            }

            /*--------------------------------------------------------------------------------------
             * Kalman prediction...
             *------------------------------------------------------------------------------------*/
            kalmanTrackedState = kalman.predict();

            if(this.objectLostCount == OBJECT_TRACK_TIMEOUT)
            {
               this.objectTrackState = State.OBJECT_LOST;
            }/*End if( this.objectLostCount == OBJECT_TRACK_TIMEOUT)*/

            break;/*End case OBJECT_TRACK:*/
         }
         case OBJECT_LOST:
            this.currentObjectTrackState = State.OBJECT_LOST;
            this.objectLostCount = 0;
            break;/*End case OBJECT_LOST:*/

         case OBJECT_IDLE:
            this.currentObjectTrackState = State.OBJECT_IDLE;
            /*--------------------------------------------------------------------------------------
             * This state is entered after boot, waiting for user to touch an object.
             *------------------------------------------------------------------------------------*/
            break;/*End case OBJECT_IDLE:*/

      }/*End switch( this.objectTrackState)*/

      /*--------------------------------------------------------------------------------------------
       * Store the last time this method was updated.
       *------------------------------------------------------------------------------------------*/
      prevNanoTime = currentNanoTime;

      /*--------------------------------------------------------------------------------------------
       * Return the center coordinates, "tracking" bounding rectangle, and average HSV color of the
       * tracked object, use this information to scale the image such that the blob detection
       * algorithm looks in a specific area for the object of interest.
       *------------------------------------------------------------------------------------------*/

   }/*End updateObjectTrack*/

   private double[] getFilteredTargetCoordsAbsolute( )
   {
      double x = kalmanTrackedState.get(0,0)[0];
      double y = kalmanTrackedState.get(1,0)[0];
      double dx = kalmanTrackedState.get(2,0)[0];
      double dy = kalmanTrackedState.get(3,0)[0];

      return new double[]{x,y,dx,dy, filteredTargetWidthHeight[0], filteredTargetWidthHeight[1]};
   }

   public double[] getFilteredTargetCoords( )
   {
      int cols = frameCols;
      int rows = frameRows;
      double x = kalmanTrackedState.get(0,0)[0] - (double)cols/2;
      double y = kalmanTrackedState.get(1,0)[0] - (double)rows/2;
      double dx = kalmanTrackedState.get(2,0)[0];
      double dy = kalmanTrackedState.get(3,0)[0];

      return new double[]{x,-y,dx,dy, filteredTargetWidthHeight[0], filteredTargetWidthHeight[1]};
   }

   public int[] getRawTargetCoords( )
   {
      if( rawTarget != null)
      {
         int x = rawTarget.x;
         int y = rawTarget.y;
         int width = rawTarget.width;
         int height = rawTarget.height;
         return new int[]{(int) x + (width / 2), (int) y + (height / 2), (int) width, (int) height};
      }
      else
         return null;
   }

   private Scalar getBlobColor(Rect blob)
   {
      int x;
      int y;
      int width;
      int height;

      Scalar mBlobColorHsv = new Scalar(255);

      Mat blobRegionRgba = currentImage.rgba().submat(blob);

      Mat blobRegionHsv = new Mat();
      Imgproc.cvtColor(blobRegionRgba, blobRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

      // Calculate average color of touched region
      mBlobColorHsv = Core.sumElems(blobRegionHsv);
      int pointCount = blob.width * blob.height;
      for(int i = 0; i < mBlobColorHsv.val.length; i++)
         mBlobColorHsv.val[i] /= pointCount;

      blobRegionRgba.release();
      blobRegionHsv.release();

      return mBlobColorHsv;
   }

   private Rect findTarget()
   {
      /*--------------------------------------------------------------------------------------------
       * Find the blob in the region that was touched.
       *------------------------------------------------------------------------------------------*/
      List<Rect> blobs = this.findBlobs(currentImage, true);

      Double area;
      int x;
      int y;
      int width;
      int height;
      boolean objectMatch = false;
      int blobNumber = 0;
      double[] kalmanCoords = getFilteredTargetCoordsAbsolute();

      double maxArea = 0;

      for(int i = 0; i < blobs.size(); i++)
      {
         //Scalar blobColor = g.getBlobColor(blobs.get(i));
         x = blobs.get(i).x;
         y = blobs.get(i).y;
         width = blobs.get(i).width;
         height = blobs.get(i).height;
         area = blobs.get(i).area();
         x = x + width/2;
         y = y + height/2;
         /*-----------------------------------------------------------------------------------------
          * If kalman center is inside the blob rect then its a match.
          *---------------------------------------------------------------------------------------*/
         //int centerx = (int)kalmanCoords[0];
         //int centery = (int)kalmanCoords[1];
         //boolean test1 = (centerx >= x) && (centerx <= (x + width));
         //boolean test2 = (centery >= y) && (centery <= (y + height));
         if(area > maxArea)
         {
            maxArea = area;
            objectMatch = true;
            blobNumber = i;
            //break;
         }
      }

      if( maxArea > 0)
      {
         rawTarget = blobs.get(blobNumber);
         return blobs.get(blobNumber);
      }
      else
      {
         rawTarget = null;
         return null;
      }
   }

   private ArrayList<Rect> findBlobs(CvCameraViewFrame currentImage, boolean useSubMat)
   {
      int cols = currentImage.rgba().cols();
      int rows = currentImage.rgba().rows();
      Rect newRect = getRegionOfInterestRect();

      if( useSubMat)
      {
         Mat trackedRgba = currentImage.rgba().submat(newRect);
         this.mDetector.process(trackedRgba);
      }
      else
         this.mDetector.process(currentImage.rgba());

      List<MatOfPoint> contours = this.mDetector.getContours();

      MatOfPoint2f approxCurve = new MatOfPoint2f();
      Double area;
      int x;
      int y;
      int width;
      int height;
      ArrayList<Rect> addresses = new ArrayList<Rect>();

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

         if( useSubMat)
         {
            rect.x = rect.x + newRect.x;
            rect.y = rect.y + newRect.y;
         }

         addresses.add(rect);
      }

      this.blobs = addresses;
      this.currentImage = currentImage;
      return this.blobs;
   }

}
