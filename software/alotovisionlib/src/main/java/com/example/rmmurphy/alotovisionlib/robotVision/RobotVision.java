package com.example.rmmurphy.alotovisionlib.robotVision;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
   private int objectLostCount;
   private static int OBJECT_TRACK_TIMEOUT = 30;
   private static double TRACK_RECT_MAX_SCALAR = 5;
   private static double HEIGHT_WIDTH_FILTER_AMOUNT = 10;
   private static double COLOR_FILTER_AMOUNT = 20;
   private static int INIT_RETRY = 5;
   private static int COLOR_RANGE_STD_MULT = 4;
   private State objectTrackState;
   private State currentObjectTrackState;
   private Scalar objectColorHsv;
   private Scalar objectColorRgb;
   private Rect regionOfInterestRect;
   private Rect initialRegionOfInterestRect;
   private Rect rawTarget;
   private int rawTargetIndex;
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
   private int initRetryCount;
   private Scalar avrTargetColorHSV;
   private Scalar avrTargetStdHSV;
   private double targetHeightToWidthRatio;

   private Mat hueHistogram;
   private Mat valHistogram;
   private Mat satHistogram;

   private MatOfFloat hsvRanges;
   private MatOfInt histChannels;
   private MatOfInt histSize;
   private ArrayList<Mat>hueList;
   private ArrayList<Mat>satList;
   private ArrayList<Mat>valList;
   private ArrayList<Mat>histImages;
   private ArrayList<Mat>equalizedImage;

   private Mat imgRgbEqualized;
   private Mat temp;
   private double[] avrHueHist;
   private double[] avrSatHist;
   private double[] avrValHist;
   private double avrResX;
   private double avrResY;

   private Mat currentRgba;

   public enum State
   {
      OBJECT_FIRST_TOUCH, OBJECT_TRACK_INIT, OBJECT_TRACK, OBJECT_LOST, OBJECT_IDLE
   }

   public ArrayList<Rect> blobRects;
   private List<MatOfPoint> blobContours;

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
      Core.setIdentity(meas);
      kalmanProcNoiseCov.put(0, 0, 1e-3);
      kalmanProcNoiseCov.put(1, 1, 1e-3);
      kalmanProcNoiseCov.put(2, 2, 1e-5);
      kalmanProcNoiseCov.put(3, 3, 1e-5);
      Core.setIdentity(kalmanMeasNoiseCov, Scalar.all(1e-3));

      /*--------------------------------------------------------------------------------------------
       * We have known starting coordinates so set this value high.
       *------------------------------------------------------------------------------------------*/
      Core.setIdentity(errorCovPost, Scalar.all(1));

      kalman.set_measurementMatrix(meas);
      kalman.set_processNoiseCov(kalmanProcNoiseCov);
      kalman.set_measurementNoiseCov(kalmanMeasNoiseCov);
      kalman.set_errorCovPost(errorCovPost);

      mDetector = new ColorBlobDetector();

      this.blobRects = new ArrayList<Rect>();
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
      initRetryCount = 0;

      avrTargetColorHSV = new Scalar(255);
      avrTargetStdHSV   = new Scalar(255);
      targetHeightToWidthRatio = 0.0f;


      hueHistogram = new Mat();
      valHistogram = new Mat();
      satHistogram = new Mat();
      hueList   = new ArrayList<Mat>();
      satList   = new ArrayList<Mat>();
      valList   = new ArrayList<Mat>();
      hsvRanges =  new MatOfFloat( 0f,256f);
      histChannels = new MatOfInt(0);
      histSize = new MatOfInt( 64);
      histImages   = new ArrayList<Mat>();
      equalizedImage = new ArrayList<Mat>();
      imgRgbEqualized = new Mat();
      temp = new Mat();
      avrHueHist = new double[64];
      avrSatHist = new double[64];
      avrValHist = new double[64];

      avrResX = 0;
      avrResY = 0;
      currentRgba = new Mat(height, width, CvType.CV_8UC4);
   }

   public int getTargetContourIndex()
   {
      return rawTargetIndex;
   }

   public List<MatOfPoint> getContours()
   {
      return blobContours;
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

   public Rect getInitialRegionOfInterestRect()
   {
      int cols = frameCols;
      int rows = frameRows;
      Rect newRet = new Rect();

      newRet = initialRegionOfInterestRect;

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

   private void setBlobs(ArrayList<Rect> blobRects)
   {
      this.blobRects = blobRects;
   }

   private ArrayList<Rect> getBlobs()
   {
      return this.blobRects;
   }/*End getBlobs*/

   private Scalar convertScalarHsv2Rgba(Scalar hsvColor)
   {
      Mat pointMatRgba = new Mat();
      Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
      Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

      return new Scalar(pointMatRgba.get(0, 0));
   }

   private Scalar convertScalarRgba2Hsv(Scalar rgbaColor)
   {
      Mat pointMatHsv = new Mat();
      Mat pointMatRgba = new Mat(1, 1, CvType.CV_8UC3, rgbaColor);
      Imgproc.cvtColor(pointMatRgba, pointMatHsv, Imgproc.COLOR_RGB2HSV_FULL, 4);

      return new Scalar(pointMatHsv.get(0, 0));
   }

   private void equalizedImage()
   {

      Imgproc.cvtColor(currentRgba, imgRgbEqualized, Imgproc.COLOR_BGR2YCrCb); //change the color image from BGR to YCrCb format
      Core.split(imgRgbEqualized, equalizedImage); //split the image into channels
      Imgproc.equalizeHist(equalizedImage.get(0), temp); //equalize histogram on the 1st channel (Y)
      equalizedImage.set(0, temp);
      Core.merge(equalizedImage, imgRgbEqualized); //merge 3 channels including the modified 1st channel into one image
      Imgproc.cvtColor(imgRgbEqualized, currentRgba, Imgproc.COLOR_YCrCb2BGR); //change the color image from YCrCb to BGR format (to

   }

   public void updateObjectTrack( Mat frameRgba)
   {
       /*-------------------------------------------------------------------------------------------
        * Store the current camera image...
        *-----------------------------------------------------------------------------------------*/
      frameRgba.copyTo(currentRgba);
      //currentRgba = frameRgba;
      int cols = currentRgba.cols();
      int rows = currentRgba.rows();
      long currentNanoTime = System.nanoTime();
      double deltaTimeSec =  (double)(currentNanoTime - prevNanoTime)  / 1000000000.0;;

      //equalizedImage();

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
            Mat touchedRegionRgba = currentRgba.submat(this.initialRegionOfInterestRect);

            //Mat touchedRegionHsv = new Mat();
            //Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

            // Calculate average color of touched region
            //this.objectColorHsv = Core.sumElems(touchedRegionHsv);
            //int pointCount = this.initialRegionOfInterestRect.width * this.initialRegionOfInterestRect.height;
            //for(int i = 0; i < this.objectColorHsv.val.length; i++)
            //   this.objectColorHsv.val[i] /= pointCount;

            //objectColorRgb = convertScalarHsv2Rgba(this.objectColorHsv);

            // Calculate average color of touched region
            this.objectColorRgb = Core.sumElems(touchedRegionRgba);
            int pointCount = this.initialRegionOfInterestRect.width * this.initialRegionOfInterestRect.height;
            for(int i = 0; i < this.objectColorRgb.val.length; i++)
               this.objectColorRgb.val[i] /= pointCount;

            mDetector.setColorRadius(new Scalar(15, 40, 40, 0));
            mDetector.setHsvColor(convertScalarRgba2Hsv(this.objectColorRgb));

            touchedRegionRgba.release();
            //touchedRegionHsv.release();

            /*--------------------------------------------------------------------------------------
             * Find the blob in the region that was touched.
             *------------------------------------------------------------------------------------*/
            List<Rect> blobRects = this.findBlobs(false);

            Double area;
            int x;
            int y;
            int width;
            int height;
            boolean objectMatch = false;
            int blobNumber = 0;

            for(int i = 0; i < blobRects.size(); i++)
            {
               x = blobRects.get(i).x;
               y = blobRects.get(i).y;

               width = blobRects.get(i).width;
               height = blobRects.get(i).height;

               x = x - width/2;
               y = y - height/2;

               int centerx = this.initialRegionOfInterestRect.x + this.initialRegionOfInterestRect.width/2;
               int centery = this.initialRegionOfInterestRect.y + this.initialRegionOfInterestRect.height/2;
               boolean test1 = (centerx >= x) && (centerx <= (x + 2*width));
               boolean test2 = (centery >= y) && (centery <= (y + 2*height));
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
               x = blobRects.get(blobNumber).x;
               y = blobRects.get(blobNumber).y;
               width = blobRects.get(blobNumber).width;
               height = blobRects.get(blobNumber).height;

               filteredTargetWidthHeight[0] = width;
               filteredTargetWidthHeight[1] = height;
               targetHeightToWidthRatio = (double)height/width;

               this.regionOfInterestRect.x = (int)((double)x + ((double) width / 2)  - ((double)width*TRACK_RECT_MAX_SCALAR/2));
               this.regionOfInterestRect.y = (int)((double)y + ((double) height / 2) - ((double)height*TRACK_RECT_MAX_SCALAR/2));
               this.regionOfInterestRect.width = (int)(width * TRACK_RECT_MAX_SCALAR);
               this.regionOfInterestRect.height = (int)(height * TRACK_RECT_MAX_SCALAR);

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

               Mat procNoise = kalman.get_processNoiseCov();
               procNoise.put(0,0, 1e-2);
               procNoise.put(1,1, 1e-2);
               kalman.set_processNoiseCov(procNoise);

               Scalar[] meanVar = getBlobColor(blobRects.get(blobNumber), true);

               avrTargetColorHSV.val[0] = meanVar[0].val[0];
               avrTargetColorHSV.val[1] = meanVar[0].val[1];
               avrTargetColorHSV.val[2] = meanVar[0].val[2];

               avrTargetColorHSV.val[3] = 0;

               avrTargetStdHSV = meanVar[1];
               avrTargetStdHSV.val[0] = meanVar[1].val[0]*COLOR_RANGE_STD_MULT;
               avrTargetStdHSV.val[1] = meanVar[1].val[1]*COLOR_RANGE_STD_MULT;
               avrTargetStdHSV.val[2] = meanVar[1].val[2]*COLOR_RANGE_STD_MULT;
               avrTargetStdHSV.val[3] = 0;

               if( avrTargetStdHSV.val[0] < 10)
                  avrTargetStdHSV.val[0] = 10;

               if( avrTargetStdHSV.val[1] < 10)
                  avrTargetStdHSV.val[1] = 10;

               if( avrTargetStdHSV.val[2] < 10)
                  avrTargetStdHSV.val[2] = 10;

               //mDetector.setColorRadius(avrTargetStdHSV);
               mDetector.setHsvColor(avrTargetColorHSV);

               /*-----------------------------------------------------------------------------------
                * We have known starting coordinates so set this value high.
                *---------------------------------------------------------------------------------*/
               Mat errorCovPost = kalman.get_errorCovPost();
               Core.setIdentity(errorCovPost, Scalar.all(1));
               kalman.set_errorCovPost(errorCovPost);
               initRetryCount = 0;
               this.objectTrackState = State.OBJECT_TRACK;
               avrResX = 0;
               avrResY = 0;
            }
            else
            {
               initRetryCount++;
               if( initRetryCount == INIT_RETRY)
               {
                  initRetryCount = 0;
                  this.objectTrackState = State.OBJECT_IDLE;
               }
            }
            break;/*End case OBJECT_TRACK_INIT:*/
         }
         case OBJECT_TRACK:
         {
            this.currentObjectTrackState = State.OBJECT_TRACK;

            /*--------------------------------------------------------------------------------------
             * Look for the target using the last known region of interest, select the winner based
             * the blob the encompasses the Kalman filtered coordinates.
             *------------------------------------------------------------------------------------*/
            Rect target = findTarget( true, false);
            int x;
            int y;
            double dx;
            double dy;

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
            kalmanTransMatrix.put(1, 3, deltaTimeSec);
            kalman.set_transitionMatrix(kalmanTransMatrix);

            if( target != null)
            {
               /*-----------------------------------------------------------------------------------
                * Kalman prediction...
                *---------------------------------------------------------------------------------*/
               kalmanTrackedState = kalman.predict();

               double measX = (double)target.x + (double)target.width/2;
               double measY = (double)target.y + (double)target.height/2;

               /*-----------------------------------------------------------------------------------
                * Kalman measurement update...
                *---------------------------------------------------------------------------------*/
               kalmanMeas.put( 0, 0, measX);
               kalmanMeas.put( 1, 0, measY);
               kalmanTrackedState = kalman.correct( kalmanMeas);
               this.objectLostCount = 0;
               /*-----------------------------------------------------------------------------------
                * Move the object tracking rectangle based on the Kalman estimate coordinates.
                *---------------------------------------------------------------------------------*/
               x = (int)kalmanTrackedState.get(0,0)[0];
               y = (int)kalmanTrackedState.get(1,0)[0];
               dx = kalmanTrackedState.get(2,0)[0];
               dy = kalmanTrackedState.get(3,0)[0];

               double xRes = Math.abs( measX - x);
               double yRes = Math.abs( measY - y);

               avrResX = avrResX*.9f + .1f*xRes;
               avrResY = avrResY*.9f + .1f*yRes;

               Mat procNoise = kalman.get_processNoiseCov();

               if( xRes > (2.0*avrResX))
               {
                  procNoise.put(0,0, procNoise.get(0,0)[0] + 1e-1);
                  //if( procNoise.get(0,0)[0] > 1)
                  //   procNoise.put(0,0, 1);
               }
               else
               {
                  procNoise.put(0,0, procNoise.get(0,0)[0] - 1e-1);
                  if( procNoise.get(0,0)[0] < 1e-2)
                     procNoise.put(0,0, 1e-2);
               }

               if( yRes > (2.0*avrResY))
               {
                  procNoise.put(1,1, procNoise.get(1,1)[0] + 1e-1);
                  //if( procNoise.get(1,1)[0] > 1)
                  //   procNoise.put(1,1, 1);
               }
               else
               {
                  procNoise.put(1,1, procNoise.get(1,1)[0] - 1e-1);
                  if( procNoise.get(1,1)[0] < 1e-2)
                     procNoise.put(1,1, 1e-2);
               }

               kalman.set_processNoiseCov(procNoise);

               filteredTargetWidthHeight[0] = filteredTargetWidthHeight[0]*( 1 - 1/HEIGHT_WIDTH_FILTER_AMOUNT) + (double)target.width*(1/HEIGHT_WIDTH_FILTER_AMOUNT);
               filteredTargetWidthHeight[1] = filteredTargetWidthHeight[1]*( 1 - 1/HEIGHT_WIDTH_FILTER_AMOUNT) + (double)target.height*(1/HEIGHT_WIDTH_FILTER_AMOUNT);

               targetHeightToWidthRatio = (double)filteredTargetWidthHeight[1]/filteredTargetWidthHeight[0];

               regionOfInterestRect.x = x - (int)(filteredTargetWidthHeight[0]*TRACK_RECT_MAX_SCALAR)/2;
               regionOfInterestRect.y = y - (int)(filteredTargetWidthHeight[1]*TRACK_RECT_MAX_SCALAR)/2;

               regionOfInterestRect.width  = (int)(filteredTargetWidthHeight[0]*TRACK_RECT_MAX_SCALAR);
               regionOfInterestRect.height = (int)(filteredTargetWidthHeight[1]*TRACK_RECT_MAX_SCALAR);

               /*-----------------------------------------------------------------------------------
                * Track the average color and color twice the variance of the target.
                *---------------------------------------------------------------------------------*/
               Scalar[] meanVar = getBlobColor(target, true);

               for(int i = 0; i < 3; i++)
               {
                  avrTargetColorHSV.val[i] = avrTargetColorHSV.val[i] * (1 - (1 / COLOR_FILTER_AMOUNT)) + meanVar[0].val[i]*(1/COLOR_FILTER_AMOUNT);
                  avrTargetStdHSV.val[i] = avrTargetStdHSV.val[i] * (1 - (1 / COLOR_FILTER_AMOUNT)) + meanVar[1].val[i]*COLOR_RANGE_STD_MULT*(1/COLOR_FILTER_AMOUNT);

               }

               if( avrTargetStdHSV.val[0] < 10)
                  avrTargetStdHSV.val[0] = 10;

               if( avrTargetStdHSV.val[1] < 10)
                  avrTargetStdHSV.val[1] = 10;

               if( avrTargetStdHSV.val[2] < 10)
                  avrTargetStdHSV.val[2] = 10;

               //mDetector.setColorRadius(avrTargetStdHSV);
               mDetector.setHsvColor(avrTargetColorHSV);
               objectColorHsv = avrTargetColorHSV;
               objectColorRgb = convertScalarHsv2Rgba(avrTargetColorHSV);

            }
            else
            {
               /*-----------------------------------------------------------------------------------
                * Blob not detected use the predicted measurement as the estimate.
                *---------------------------------------------------------------------------------*/
               this.objectLostCount++;
            }

            if(this.objectLostCount == OBJECT_TRACK_TIMEOUT)
            {
               this.objectTrackState = State.OBJECT_LOST;
               //mDetector.setColorRadius( new Scalar(15,30,30,0));
            }/*End if( this.objectLostCount == OBJECT_TRACK_TIMEOUT)*/

            break;/*End case OBJECT_TRACK:*/
         }
         case OBJECT_LOST:
            Double area;
            int x;
            int y;
            int width;
            int height;

            this.currentObjectTrackState = State.OBJECT_LOST;
            this.objectLostCount = 0;
            /*--------------------------------------------------------------------------------------
             * Look for the target using the entire camera frame, select the winner based on maximum
             * size.
             *------------------------------------------------------------------------------------*/
            Rect target = findTarget( false, false);

            if( target != null)
            {
               /*-----------------------------------------------------------------------------------
                * Object is a match, set the tracking rect to a size 1.5 times larger that the
                * object rectangle
                *---------------------------------------------------------------------------------*/
               x = target.x;
               y = target.y;
               width = target.width;
               height = target.height;

               filteredTargetWidthHeight[0] = width;
               filteredTargetWidthHeight[1] = height;
               targetHeightToWidthRatio = (double)height/width;

               this.regionOfInterestRect.x = (int)((double)x + ((double) width / 2)  - ((double)width*TRACK_RECT_MAX_SCALAR/2));
               this.regionOfInterestRect.y = (int)((double)y + ((double) height / 2) - ((double)height*TRACK_RECT_MAX_SCALAR/2));
               this.regionOfInterestRect.width = (int)(width * TRACK_RECT_MAX_SCALAR);
               this.regionOfInterestRect.height = (int)(height * TRACK_RECT_MAX_SCALAR);

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

               Mat procNoise = kalman.get_processNoiseCov();
               procNoise.put(0, 0, 1e-2);
               procNoise.put(1, 1, 1e-2);
               kalman.set_processNoiseCov(procNoise);

               /*-----------------------------------------------------------------------------------
                * Update transition matrix dt
                *---------------------------------------------------------------------------------*/
               kalmanTransMatrix.put(0, 2, deltaTimeSec);
               kalmanTransMatrix.put(1, 3, deltaTimeSec);
               kalman.set_transitionMatrix(kalmanTransMatrix);

               Scalar[] meanVar = getBlobColor(target, true);

               avrTargetColorHSV = meanVar[0];
               avrTargetColorHSV.val[0] = meanVar[0].val[0];
               avrTargetColorHSV.val[1] = meanVar[0].val[1];
               avrTargetColorHSV.val[2] = meanVar[0].val[2];
               avrTargetColorHSV.val[3] = 0;
               avrTargetStdHSV.val[0] = meanVar[1].val[0]*COLOR_RANGE_STD_MULT;
               avrTargetStdHSV.val[1] = meanVar[1].val[1]*COLOR_RANGE_STD_MULT;
               avrTargetStdHSV.val[2] = meanVar[1].val[2]*COLOR_RANGE_STD_MULT;
               avrTargetStdHSV.val[3] = 0;

               if( avrTargetStdHSV.val[0] < 10)
                  avrTargetStdHSV.val[0] = 10;

               if( avrTargetStdHSV.val[1] < 10)
                  avrTargetStdHSV.val[1] = 10;

               if( avrTargetStdHSV.val[2] < 10)
                  avrTargetStdHSV.val[2] = 10;

               //mDetector.setColorRadius(avrTargetStdHSV);
               mDetector.setHsvColor(avrTargetColorHSV);

               /*-----------------------------------------------------------------------------------
                * We have known starting coordinates so set this value high.
                *---------------------------------------------------------------------------------*/
               Mat errorCovPost = kalman.get_errorCovPost();
               Core.setIdentity(errorCovPost, Scalar.all(1));
               kalman.set_errorCovPost(errorCovPost);
               initRetryCount = 0;
               avrResX = 0;
               avrResY = 0;
               this.objectTrackState = State.OBJECT_TRACK;
            }

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

   public double[] getFilteredTargetCoordsAbsolute( )
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

   private void computeHistograms( Mat imageHsv)
   {
      Core.split(imageHsv, histImages);

      Mat mH = histImages.get(0);
      Mat mS = histImages.get(1);
      Mat mV = histImages.get(2);
      hueList.add(mH);
      satList.add(mS);
      valList.add(mV);

      Imgproc.calcHist(hueList,
              histChannels,
              new Mat(),
              hueHistogram,
              histSize,
              hsvRanges,
              false);

      Imgproc.calcHist(satList,
              histChannels,
              new Mat(),
              satHistogram,
              histSize,
              hsvRanges,
              false);

      Imgproc.calcHist(valList,
              histChannels,
              new Mat(),
              valHistogram,
              histSize,
              hsvRanges,
              false);

      //Core.normalize(hueHistogram, hueHistogram);
      //Core.normalize(satHistogram, satHistogram);
      //Core.normalize(valHistogram, valHistogram);

      double[] cumHue = new double[64];
      double[] cumSat = new double[64];
      double[] cumVal = new double[64];

      for( int index = 0; index < 64; index++)
      {
         avrHueHist[index] = avrHueHist[index]*(1 - (1 / COLOR_FILTER_AMOUNT)) + hueHistogram.get(index,0)[0]*(1/COLOR_FILTER_AMOUNT);
         avrSatHist[index] = avrSatHist[index]*(1 - (1 / COLOR_FILTER_AMOUNT)) + satHistogram.get(index,0)[0]*(1/COLOR_FILTER_AMOUNT);
         avrValHist[index] = avrValHist[index]*(1 - (1 / COLOR_FILTER_AMOUNT)) + valHistogram.get(index,0)[0]*(1/COLOR_FILTER_AMOUNT);
      }
      double sumHue = 0;
      double sumSat = 0;
      double sumVal = 0;

      for( int index = 0; index < 64; index++)
      {
         sumHue = 0.0f;
         sumSat = 0.0f;
         sumVal = 0.0f;
         for (int index2 = 0; index2 < index; index2++)
         {
            sumHue += avrHueHist[index2];
            sumSat += avrSatHist[index2];
            sumVal += avrValHist[index2];
         }

         cumHue[index] = sumHue;
         cumSat[index] = sumSat;
         cumVal[index] = sumVal;
      }

      double maxHue = cumHue[63];
      double hueLow = 0;
      double hueHigh = 0;
      for( int index = 0; index < 64; index++)
      {
         if( (cumHue[index] / maxHue) >= .01f)
         {
            hueLow = index;
            break;
         }

      }

      for( int index = 0; index < 64; index++)
      {
         if( (cumHue[index] / maxHue) >= .99f)
         {
            hueHigh = index;
            break;
         }

      }

      double maxSat = cumSat[63];
      double satLow = 0;
      double satHigh = 0;
      for( int index = 0; index < 64; index++)
      {
         if( (cumSat[index] / maxSat) >= .01f)
         {
            satLow = index;
            break;
         }

      }

      for( int index = 0; index < 64; index++)
      {
         if( (cumSat[index] / maxSat) >= .99f)
         {
            satHigh = index;
            break;
         }

      }

      double maxVal = cumVal[63];
      double valLow = 0;
      double valHigh = 0;
      for( int index = 0; index < 64; index++)
      {
         if( (cumVal[index] / maxVal) >= .01f)
         {
            valLow = index;
            break;
         }

      }

      for( int index = 0; index < 64; index++)
      {
         if( (cumVal[index] / maxVal) >= .99f)
         {
            valHigh = index;
            break;
         }

      }

      mDetector.setHsvLowerBound( convertScalarRgba2Hsv(new Scalar(hueLow * 4, satLow * 4, 0, 0)));
      mDetector.setHsvUpperBound(convertScalarRgba2Hsv(new Scalar(hueHigh * 4, satHigh * 4, 255, 255)));

   }

   private Scalar[] getBlobColor(Rect blob, boolean trackRect)
   {
      int x;
      int y;
      int width;
      int height;
      int cols = currentRgba.cols();
      int rows = currentRgba.rows();
      Scalar mBlobColorHsv = new Scalar(255);
      double coords[] = getFilteredTargetCoordsAbsolute();
      // Calculate average color of touched region
      MatOfDouble tempMean = new MatOfDouble();
      MatOfDouble tempStd = new MatOfDouble();

      if( trackRect)
      {

         x = (int) coords[0];
         y = (int) coords[1];
         width = (int) coords[4];
         height = (int) coords[5];

         x = x - (int) (width / TRACK_RECT_MAX_SCALAR) / 2;
         y = y - (int) (height / TRACK_RECT_MAX_SCALAR) / 2;
         width = (int) ((double) (width / TRACK_RECT_MAX_SCALAR));
         if (width < 8)
            width = 8;
         height = (int) ((double) (height / TRACK_RECT_MAX_SCALAR));
         if (height < 8)
            height = 8;

         if (x < 0)
            x = 0;

         if (x + width > cols)
            width = cols - x;

         if (y < 0)
            y = 0;

         if (y + height > rows)
            height = rows - y;

         Rect boundingRect = new Rect();
         boundingRect.x = x;
         boundingRect.y = y;
         boundingRect.width = width;
         boundingRect.height = height;

         Mat blobRegionRgba = currentRgba.submat(boundingRect);

         //Mat blobRegionHsv = new Mat();
         //Imgproc.cvtColor(blobRegionRgba, blobRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

         //blobRegionRgba = equalizedImage(blobRegionRgba);

         Core.meanStdDev(blobRegionRgba, tempMean, tempStd);

         Scalar temp = new Scalar(255);
         Scalar temp2 = new Scalar(255);
         Scalar rgblow = new Scalar(255);
         Scalar rgbhigh = new Scalar(255);
         Scalar huelow = new Scalar(255);
         Scalar huehigh = new Scalar(255);
         Scalar huerange = new Scalar(255);

         temp.val[0] = tempMean.get(0, 0)[0];
         temp.val[1] = tempMean.get(1, 0)[0];
         temp.val[2] = tempMean.get(2, 0)[0];

         temp2.val[0] = tempStd.get(0, 0)[0];
         temp2.val[1] = tempStd.get(1, 0)[0];
         temp2.val[2] = tempStd.get(2, 0)[0];

         rgblow.val[0] = temp.val[0] - temp2.val[0];
         if( rgblow.val[0] < 0)
            rgblow.val[0] = 0;
         rgblow.val[1] = temp.val[1] - temp2.val[1];
         if( rgblow.val[1] < 0)
            rgblow.val[1] = 0;
         rgblow.val[2] = temp.val[2] - temp2.val[2];
         if( rgblow.val[2] < 0)
            rgblow.val[2] = 0;

         rgbhigh.val[0] = temp.val[0] + temp2.val[0];
         if( rgbhigh.val[0] > 255)
            rgbhigh.val[0] = 255;
         rgbhigh.val[1] = temp.val[1] + temp2.val[1];
         if( rgbhigh.val[1] > 255)
            rgbhigh.val[1] = 255;
         rgbhigh.val[2] = temp.val[2] + temp2.val[2];
         if( rgbhigh.val[2] > 255)
            rgbhigh.val[2] = 255;

         huelow = convertScalarRgba2Hsv(rgblow);
         huehigh = convertScalarRgba2Hsv(rgbhigh);

         huerange.val[0] = huehigh.val[0] - huelow.val[0];
         huerange.val[1] = huehigh.val[1] - huelow.val[1];
         huerange.val[2] = huehigh.val[2] - huelow.val[2];

         //if( currentObjectTrackState == State.OBJECT_TRACK)
         //computeHistograms(blobRegionRgba);

         blobRegionRgba.release();
         //blobRegionHsv.release();

         return new Scalar[]{convertScalarRgba2Hsv(temp),huerange};
      }
      else
      {
         Rect newRect = getRegionOfInterestRect();
         Mat blobRegionRgba2 = currentRgba.submat(newRect);

         //Mat blobRegionHsv2 = new Mat();
         //Imgproc.cvtColor(blobRegionRgba2, blobRegionHsv2, Imgproc.COLOR_RGB2HSV_FULL);

         //blobRegionRgba2 = equalizedImage(blobRegionRgba2);

         Mat mask = new Mat(new Size(blobRegionRgba2.cols(), blobRegionRgba2.rows()), CvType.CV_8UC1);

         mask.setTo(new Scalar(0.0));
         Imgproc.drawContours(mask, blobContours, rawTargetIndex, new Scalar(255), -1);

         Core.meanStdDev(blobRegionRgba2, tempMean, tempStd, mask);

         Scalar temp = new Scalar(255);
         Scalar temp2 = new Scalar(255);
         Scalar rgblow = new Scalar(255);
         Scalar rgbhigh = new Scalar(255);
         Scalar huelow = new Scalar(255);
         Scalar huehigh = new Scalar(255);
         Scalar huerange = new Scalar(255);

         temp.val[0] = tempMean.get(0, 0)[0];
         temp.val[1] = tempMean.get(1, 0)[0];
         temp.val[2] = tempMean.get(2, 0)[0];

         temp2.val[0] = tempStd.get(0, 0)[0];
         temp2.val[1] = tempStd.get(1, 0)[0];
         temp2.val[2] = tempStd.get(2, 0)[0];

         rgblow.val[0] = temp.val[0] - temp2.val[0];
         if( rgblow.val[0] < 0)
            rgblow.val[0] = 0;
         rgblow.val[1] = temp.val[1] - temp2.val[1];
         if( rgblow.val[1] < 0)
            rgblow.val[1] = 0;
         rgblow.val[2] = temp.val[2] - temp2.val[2];
         if( rgblow.val[2] < 0)
            rgblow.val[2] = 0;

         rgbhigh.val[0] = temp.val[0] + temp2.val[0];
         if( rgbhigh.val[0] > 255)
            rgbhigh.val[0] = 255;
         rgbhigh.val[1] = temp.val[1] + temp2.val[1];
         if( rgbhigh.val[1] > 255)
            rgbhigh.val[1] = 255;
         rgbhigh.val[2] = temp.val[2] + temp2.val[2];
         if( rgbhigh.val[2] > 255)
            rgbhigh.val[2] = 255;

         huelow = convertScalarRgba2Hsv(rgblow);
         huehigh = convertScalarRgba2Hsv(rgbhigh);

         huerange.val[0] = huehigh.val[0] - huelow.val[0];
         huerange.val[1] = huehigh.val[1] - huelow.val[1];
         huerange.val[2] = huehigh.val[2] - huelow.val[2];

         //if( currentObjectTrackState == State.OBJECT_TRACK)
         //computeHistograms(blobRegionRgba);

         blobRegionRgba2.release();
         //blobRegionHsv.release();

         return new Scalar[]{convertScalarRgba2Hsv(temp),huerange};
      }
   }

   private Rect findTarget( boolean useROI, boolean useKalman)
   {
      /*--------------------------------------------------------------------------------------------
       * Find the blob in the region that was touched.
       *------------------------------------------------------------------------------------------*/
      List<Rect> blobRects = this.findBlobs(useROI);

      Double area;
      int x;
      int y;
      int width;
      int height;
      boolean objectMatch = false;
      int blobNumber = 0;
      double[] kalmanCoords = getFilteredTargetCoordsAbsolute();

      double maxArea = 0.0f;
      int widthHeightScalar = 1;

      for(int i = 0; i < blobRects.size(); i++)
      {

         x = blobRects.get(i).x;
         y = blobRects.get(i).y;
         width = blobRects.get(i).width;
         height = blobRects.get(i).height;
         area   = blobRects.get(i).area();
         /*-----------------------------------------------------------------------------------------
          * Extend blob area for detection purposes.
          *---------------------------------------------------------------------------------------*/
         x = x - width*widthHeightScalar;
         y = y - height*widthHeightScalar;

         if( useKalman)
         {
            if (((kalmanCoords[0] >= x) && (kalmanCoords[0] <= (x + 3*widthHeightScalar*width))) && ((kalmanCoords[1] >= y) && (kalmanCoords[1] <= (y + 3*widthHeightScalar*height)))) {
               objectMatch = true;
               blobNumber = i;
               break;
            }
         }
         else
         {
            if( area > maxArea)
            {
               objectMatch = true;
               maxArea = area;
               blobNumber = i;
            }
         }
      }

      if( objectMatch)
      {
         rawTarget = blobRects.get(blobNumber);
         rawTargetIndex = blobNumber;
         return blobRects.get(blobNumber);
      }
      else
      {
         rawTarget = null;
         return null;
      }
   }

   private ArrayList<Rect> findBlobs(boolean useSubMat)
   {
      int cols = currentRgba.cols();
      int rows = currentRgba.rows();
      Rect newRect = getRegionOfInterestRect();

      if( useSubMat)
      {
         Mat trackedRgba = currentRgba.submat(newRect);
         this.mDetector.process(trackedRgba);
      }
      else
         this.mDetector.process(currentRgba);

      blobContours = this.mDetector.getContours();

      MatOfPoint2f approxCurve = new MatOfPoint2f();
      Double area;
      int x;
      int y;
      int width;
      int height;
      ArrayList<Rect> addresses = new ArrayList<Rect>();

      for(int i = 0; i < blobContours.size(); i++)
      {
         //Convert contours(i) from MatOfPoint to MatOfPoint2f
         MatOfPoint2f contour2f = new MatOfPoint2f(blobContours.get(i).toArray());
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

      this.blobRects = addresses;
      return this.blobRects;
   }

}
