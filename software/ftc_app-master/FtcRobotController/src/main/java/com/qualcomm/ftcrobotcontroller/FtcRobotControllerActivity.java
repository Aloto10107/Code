/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller;

import android.app.ActionBar;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.hardware.usb.UsbManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.IBinder;
import android.preference.PreferenceManager;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.appindexing.Action;
import com.google.android.gms.appindexing.AppIndex;
import com.google.android.gms.common.api.GoogleApiClient;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftcrobotcontroller.opmodes.FtcOpModeRegister;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.wifi.WifiDirectAssistant;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.Serializable;

import java.util.ArrayList;
import java.util.List;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;
import android.view.SurfaceView;

import static org.opencv.imgproc.Imgproc.boundingRect;

public class FtcRobotControllerActivity extends Activity implements OnTouchListener, CvCameraViewListener2 {

    private static final int REQUEST_CONFIG_WIFI_CHANNEL = 1;
    private static final boolean USE_DEVICE_EMULATION = false;
    private static final int NUM_GAMEPADS = 2;

    public static final String CONFIGURE_FILENAME = "CONFIGURE_FILENAME";

    protected SharedPreferences preferences;

    protected UpdateUI.Callback callback;
    protected Context context;
    private Utility utility;
    protected ImageButton buttonMenu;

    protected TextView textDeviceName;
    protected TextView textWifiDirectStatus;
    protected TextView textRobotStatus;
    protected TextView[] textGamepad = new TextView[NUM_GAMEPADS];
    protected TextView textOpMode;
    protected TextView textErrorMessage;
    protected ImmersiveMode immersion;

    protected UpdateUI updateUI;
    protected Dimmer dimmer;
    protected LinearLayout entireScreenLayout;

    protected FtcRobotControllerService controllerService;

    protected FtcEventLoop eventLoop;

    private boolean mIsColorSelected = false;
    private Mat mRgba;
    private Scalar mBlobColorRgba;
    private Scalar mBlobColorHsv;
    private ColorBlobDetector mDetector;
    private Mat mSpectrum;
    private Size SPECTRUM_SIZE;
    private Scalar CONTOUR_COLOR;

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(FtcRobotControllerActivity.this);
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };
    /**
     * ATTENTION: This was auto-generated to implement the App Indexing API.
     * See https://g.co/AppIndexing/AndroidStudio for more information.
     */
    private GoogleApiClient client;

    protected class RobotRestarter implements Restarter {

        public void requestRestart() {
            requestRobotRestart();
        }

    }

    protected ServiceConnection connection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            FtcRobotControllerBinder binder = (FtcRobotControllerBinder) service;
            onServiceBind(binder.getService());
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            controllerService = null;
        }
    };

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        if (UsbManager.ACTION_USB_ACCESSORY_ATTACHED.equals(intent.getAction())) {
            // a new USB device has been attached
            DbgLog.msg("USB Device attached; app restart may be needed");
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_ftc_controller);

        utility = new Utility(this);
        context = this;
        entireScreenLayout = (LinearLayout) findViewById(R.id.entire_screen);
        buttonMenu = (ImageButton) findViewById(R.id.menu_buttons);
        buttonMenu.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                openOptionsMenu();
            }
        });

        textDeviceName = (TextView) findViewById(R.id.textDeviceName);
        textWifiDirectStatus = (TextView) findViewById(R.id.textWifiDirectStatus);
        textRobotStatus = (TextView) findViewById(R.id.textRobotStatus);
        textOpMode = (TextView) findViewById(R.id.textOpMode);
        textErrorMessage = (TextView) findViewById(R.id.textErrorMessage);
        textGamepad[0] = (TextView) findViewById(R.id.textGamepad1);
        textGamepad[1] = (TextView) findViewById(R.id.textGamepad2);
        immersion = new ImmersiveMode(getWindow().getDecorView());
        dimmer = new Dimmer(this);
        dimmer.longBright();
        Restarter restarter = new RobotRestarter();

        updateUI = new UpdateUI(this, dimmer);
        updateUI.setRestarter(restarter);
        updateUI.setTextViews(textWifiDirectStatus, textRobotStatus,
                textGamepad, textOpMode, textErrorMessage, textDeviceName);
        callback = updateUI.new Callback();

        PreferenceManager.setDefaultValues(this, R.xml.preferences, false);
        preferences = PreferenceManager.getDefaultSharedPreferences(this);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);

        hittingMenuButtonBrightensScreen();

        if (USE_DEVICE_EMULATION) {
            HardwareFactory.enableDeviceEmulation();
        }
        // ATTENTION: This was auto-generated to implement the App Indexing API.
        // See https://g.co/AppIndexing/AndroidStudio for more information.
        client = new GoogleApiClient.Builder(this).addApi(AppIndex.API).build();
    }

    @Override
    protected void onStart() {
        super.onStart();
        // ATTENTION: This was auto-generated to implement the App Indexing API.
        // See https://g.co/AppIndexing/AndroidStudio for more information.
        client.connect();

        // save 4MB of logcat to the SD card
        RobotLog.writeLogcatToDisk(this, 4 * 1024);

        Intent intent = new Intent(this, FtcRobotControllerService.class);
        bindService(intent, connection, Context.BIND_AUTO_CREATE);

        utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);

        callback.wifiDirectUpdate(WifiDirectAssistant.Event.DISCONNECTED);

        entireScreenLayout.setOnTouchListener(new OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                dimmer.handleDimTimer();
                return false;
            }
        });

        // ATTENTION: This was auto-generated to implement the App Indexing API.
        // See https://g.co/AppIndexing/AndroidStudio for more information.
        Action viewAction = Action.newAction(
                Action.TYPE_VIEW, // TODO: choose an action type.
                "FtcRobotController Page", // TODO: Define a title for the content shown.
                // TODO: If you have web page content that matches this app activity's content,
                // make sure this auto-generated web page URL is correct.
                // Otherwise, set the URL to null.
                Uri.parse("http://host/path"),
                // TODO: Make sure this auto-generated app deep link URI is correct.
                Uri.parse("android-app://com.qualcomm.ftcrobotcontroller/http/host/path")
        );
        AppIndex.AppIndexApi.start(client, viewAction);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    protected void onStop() {
        super.onStop();
        // ATTENTION: This was auto-generated to implement the App Indexing API.
        // See https://g.co/AppIndexing/AndroidStudio for more information.
        Action viewAction = Action.newAction(
                Action.TYPE_VIEW, // TODO: choose an action type.
                "FtcRobotController Page", // TODO: Define a title for the content shown.
                // TODO: If you have web page content that matches this app activity's content,
                // make sure this auto-generated web page URL is correct.
                // Otherwise, set the URL to null.
                Uri.parse("http://host/path"),
                // TODO: Make sure this auto-generated app deep link URI is correct.
                Uri.parse("android-app://com.qualcomm.ftcrobotcontroller/http/host/path")
        );
        AppIndex.AppIndexApi.end(client, viewAction);

        if (controllerService != null) unbindService(connection);

        RobotLog.cancelWriteLogcatToDisk(this);
        // ATTENTION: This was auto-generated to implement the App Indexing API.
        // See https://g.co/AppIndexing/AndroidStudio for more information.
        client.disconnect();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        SPECTRUM_SIZE = new Size(100, 35);
        CONTOUR_COLOR = new Scalar(0, 255, 0, 255);
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public boolean onTouch(View v, MotionEvent event)
    {
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        //Get a handle to the robot vision global variable
        RobotVision g = (RobotVision) getApplication();

        int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
        int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;

        int x = (int) event.getX() - xOffset;
        int y = (int) event.getY() - yOffset;

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

        Rect touchedRect = new Rect();

        touchedRect.x = (x > 4) ? x - 4 : 0;
        touchedRect.y = (y > 4) ? y - 4 : 0;

        touchedRect.width = (x + 4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y + 4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

        mIsColorSelected = true;

        /*------------------------------------------------------------------------------------------
         * Store the Set the object tracker to the initialization state. On the next camera frame event this
         * state will be entered.
         *----------------------------------------------------------------------------------------*/
        g.setObjectTrackingRect( touchedRect);
        g.setObjectTrackState( RobotVision.State.OBJECT_TRACK_INIT);

        return false; // don't need subsequent touch events
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame)
    {
        mRgba = inputFrame.rgba();

        //Get a handle to the robot vision global variable
        RobotVision g = (RobotVision) getApplication();

        /*------------------------------------------------------------------------------------------
         * Track the color, coordinates, and area of the selected object.
         *----------------------------------------------------------------------------------------*/
        g.updateObjectTrack( mDetector,
                             inputFrame,
                             mSpectrum,
                             SPECTRUM_SIZE);

        if( g.getObjectTrackState() == RobotVision.State.OBJECT_TRACK)
        {

            List<List<Double>> blobs = g.getBlobs();

            Double area;
            int x;
            int y;
            int width;
            int height;

            //Plot the blob locations
            for (int i = 0; i < blobs.size(); i++) {
                //Scalar blobColor = g.getBlobColor(blobs.get(i));
                x = blobs.get(i).get(1).intValue();
                y = blobs.get(i).get(3).intValue();
                width = blobs.get(i).get(4).intValue();
                height = blobs.get(i).get(5).intValue();
                Imgproc.rectangle(mRgba, new Point(x, y), new Point(x + width, y + height), new Scalar(0, 255, 0, 255), 3);
                x = x + width / 2;
                y = y + height / 2;
                area = blobs.get(i).get(6);

                Imgproc.putText(mRgba, "[" + blobs.get(i).get(0).intValue() + "," + blobs.get(i).get(2).intValue() + "," + area.intValue() + "]", new Point(x + 4, y), Core.FONT_HERSHEY_PLAIN, 2, new Scalar(255, 255, 255, 255), 3);
                Imgproc.circle(mRgba, new Point(x, y), 5, new Scalar(0, 255, 0, 255), -1);
            }

            Mat colorLabel = mRgba.submat(4, 40, 4, 40);
            colorLabel.setTo(g.getObjectColorRgb());

            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);

        }/*End if( g.getObjectTrackState() == RobotVision.State.OBJECT_TRACK)*/

        return mRgba;
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        // When the window loses focus (e.g., the action overflow is shown),
        // cancel any pending hide action. When the window gains focus,
        // hide the system UI.
        if (hasFocus) {
            if (ImmersiveMode.apiOver19()) {
                // Immersive flag only works on API 19 and above.
                immersion.hideSystemUI();
            }
        } else {
            immersion.cancelSystemUIHide();
        }
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.ftc_robot_controller, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.action_restart_robot:
                dimmer.handleDimTimer();
                Toast.makeText(context, "Restarting Robot", Toast.LENGTH_SHORT).show();
                requestRobotRestart();
                return true;
            case R.id.action_settings:
                // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
                Intent settingsIntent = new Intent("com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity.intent.action.Launch");
                startActivityForResult(settingsIntent, LaunchActivityConstantsList.FTC_ROBOT_CONTROLLER_ACTIVITY_CONFIGURE_ROBOT);
                return true;
            case R.id.action_about:
                // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
                Intent intent = new Intent("com.qualcomm.ftccommon.configuration.AboutActivity.intent.action.Launch");
                startActivity(intent);
                return true;
            case R.id.action_exit_app:
                finish();
                return true;
            case R.id.action_view_logs:
                // The string to launch this activity must match what's in AndroidManifest of FtcCommon for this activity.
                Intent viewLogsIntent = new Intent("com.qualcomm.ftccommon.ViewLogsActivity.intent.action.Launch");
                viewLogsIntent.putExtra(LaunchActivityConstantsList.VIEW_LOGS_ACTIVITY_FILENAME, RobotLog.getLogFilename(this));
                startActivity(viewLogsIntent);
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        // don't destroy assets on screen rotation
    }

    @Override
    protected void onActivityResult(int request, int result, Intent intent) {
        if (request == REQUEST_CONFIG_WIFI_CHANNEL) {
            if (result == RESULT_OK) {
                Toast toast = Toast.makeText(context, "Configuration Complete", Toast.LENGTH_LONG);
                toast.setGravity(Gravity.CENTER, 0, 0);
                showToast(toast);
            }
        }
        if (request == LaunchActivityConstantsList.FTC_ROBOT_CONTROLLER_ACTIVITY_CONFIGURE_ROBOT) {
            if (result == RESULT_OK) {
                Serializable extra = intent.getSerializableExtra(FtcRobotControllerActivity.CONFIGURE_FILENAME);
                if (extra != null) {
                    utility.saveToPreferences(extra.toString(), R.string.pref_hardware_config_filename);
                    utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
                }
            }
        }
    }

    public void onServiceBind(FtcRobotControllerService service) {
        DbgLog.msg("Bound to Ftc Controller Service");
        controllerService = service;
        updateUI.setControllerService(controllerService);

        callback.wifiDirectUpdate(controllerService.getWifiDirectStatus());
        callback.robotUpdate(controllerService.getRobotStatus());
        requestRobotSetup();
    }

    private void requestRobotSetup() {
        if (controllerService == null) return;

        FileInputStream fis = fileSetup();
        // if we can't find the file, don't try and build the robot.
        if (fis == null) {
            return;
        }

        HardwareFactory factory;

        // Modern Robotics Factory for use with Modern Robotics hardware
        HardwareFactory modernRoboticsFactory = new HardwareFactory(context);
        modernRoboticsFactory.setXmlInputStream(fis);
        factory = modernRoboticsFactory;

        eventLoop = new FtcEventLoop(factory, new FtcOpModeRegister(), callback, this);

        controllerService.setCallback(callback);
        controllerService.setupRobot(eventLoop);
    }

    private FileInputStream fileSetup() {

        final String filename = Utility.CONFIG_FILES_DIR
                + utility.getFilenameFromPrefs(R.string.pref_hardware_config_filename, Utility.NO_FILE) + Utility.FILE_EXT;

        FileInputStream fis;
        try {
            fis = new FileInputStream(filename);
        } catch (FileNotFoundException e) {
            String msg = "Cannot open robot configuration file - " + filename;
            utility.complainToast(msg, context);
            DbgLog.msg(msg);
            utility.saveToPreferences(Utility.NO_FILE, R.string.pref_hardware_config_filename);
            fis = null;
        }
        utility.updateHeader(Utility.NO_FILE, R.string.pref_hardware_config_filename, R.id.active_filename, R.id.included_header);
        return fis;
    }

    private void requestRobotShutdown() {
        if (controllerService == null) return;
        controllerService.shutdownRobot();
    }

    private void requestRobotRestart() {
        requestRobotShutdown();
        requestRobotSetup();
    }

    protected void hittingMenuButtonBrightensScreen() {
        ActionBar actionBar = getActionBar();
        if (actionBar != null) {
            actionBar.addOnMenuVisibilityListener(new ActionBar.OnMenuVisibilityListener() {
                @Override
                public void onMenuVisibilityChanged(boolean isVisible) {
                    if (isVisible) {
                        dimmer.handleDimTimer();
                    }
                }
            });
        }
    }

    public void showToast(final Toast toast) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                toast.show();
            }
        });
    }
}
