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

package org.usfirst.ftc.exampleteam.yourcodehere;

import com.example.rmmurphy.alotovisionlib.android.Cameras;
import com.qualcomm.ftcrobotcontroller.opmodes.VisionOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Rect;
import org.swerverobotics.library.ClassFactory;

import java.util.ArrayList;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class AlotoAutonomous extends VisionOpMode
{

    private double objectXCoord;
    private double objectYCoord;
    private double objectdx;
    private double objectdy;
    private double objectWidth;
    private double objectHeight;
    private double objectArea;
    private double Kp;
    private double Ki;
    private double Kd;
    private double KiIntegral;
    private double rotationalPower;
    private double linearPower;
    private double xCoordinateSetpoint;
    private double yCoordinateSetpoint;
    private double prevError;

    private enum State
    {
        DRIVE_TO_FRONT_OF_RAMP, INITIALZE_SETPOINT, DRIVE_TO_RAMP, FIND_RAMP, LOST_ROBOT, IDLE
    }

    private State robotState;

    DcMotor FrontmotorRight;
    DcMotor FrontmotorLeft;
    DcMotor BackmotorRight;
    DcMotor BackmotorLeft;

    @Override
    public void init()
    {
        super.init();
        FrontmotorRight = hardwareMap.dcMotor.get("motor_fr");
        FrontmotorLeft  = hardwareMap.dcMotor.get("motor_fl");
        BackmotorRight  = hardwareMap.dcMotor.get("motor_br");
        BackmotorLeft   = hardwareMap.dcMotor.get("motor_bl");

        FrontmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BackmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BackmotorRight.setDirection(DcMotor.Direction.REVERSE);

        ClassFactory.createEasyMotorController(this, FrontmotorLeft, BackmotorLeft);
        ClassFactory.createEasyMotorController(this, FrontmotorRight, BackmotorRight);

        resetDriveEncoders();

        //this.setCamera(Cameras.PRIMARY);
        //this.setFrameSize(new Size(400, 400));

        Ki = .0001f;
        Kp = .0005f;
        Kd = 0.0000001f;

        xCoordinateSetpoint = 0.0f;
        yCoordinateSetpoint = 0.0f;
        resetPIDController();
        robotState = State.INITIALZE_SETPOINT;
    }

    @Override
    public void loop()
    {
        super.loop();

        int[] rawTarget         = rbVis.getRawTargetCoords();
        double[] filteredTarget = rbVis.getFilteredTargetCoords();
        int leftCount           = getLeftEncoderCount();
        int rightCount          = getRightEncoderCount();
        objectXCoord            = filteredTarget[0]; /*Target 'x' position on camera screen*/
        objectYCoord            = filteredTarget[1]; /*Target 'y' position on camera screen*/
        objectdx                = filteredTarget[2]; /*The speed of the target on the x axis (pixels/sec)*/
        objectdy                = filteredTarget[3]; /*The speed of the target on the y axis (pixels/sec)*/
        objectWidth             = filteredTarget[4]; /*The width of the target as seen by the camera*/
        objectHeight            = filteredTarget[5]; /*The height of the target as seen by the camera*/
        objectArea              = objectWidth*objectHeight;

        switch(robotState)
        {
            case DRIVE_TO_FRONT_OF_RAMP:
            {
                /*----------------------------------------------------------------------------------
                 * Use encoders to drive the robot to a distance in front of the ramp...
                 *--------------------------------------------------------------------------------*/
            }
            case FIND_RAMP:
            {
                /*----------------------------------------------------------------------------------
                 * Search for the ramp by turning the robot left or right depending on whether we
                 * are on the red or blue alliance.
                 *--------------------------------------------------------------------------------*/
                if( rbVis.isTargetLocked() == false)
                {

                }
                else if( rbVis.isTargetLocked() == true)
                {
                    //robotState = State.INITIALZE_SETPOINT;
                }
            }
            case INITIALZE_SETPOINT:
            {
                resetDriveEncoders();
                stopRobot();
                if( rbVis.isTargetLocked() == true)
                {
                    xCoordinateSetpoint = objectXCoord;
                    robotState = State.DRIVE_TO_RAMP;
                }
                else if( rbVis.isTargetLocked() == false)
                {
                    robotState = State.FIND_RAMP;
                }
            }
            case DRIVE_TO_RAMP:
            {
                if( rbVis.isTargetLocked() == true)
                {
                    /*------------------------------------------------------------------------------
                     * Automonous drives in reverse!
                     *----------------------------------------------------------------------------*/
                    linearPower = -.2f;
                    /*------------------------------------------------------------------------------
                     * Smooth out the error in the 'x' position before using it to turn the robot
                     * left or right.
                     *----------------------------------------------------------------------------*/
                    rotationalPower = motorPIDController(xCoordinateSetpoint, objectXCoord);

                    /*------------------------------------------------------------------------------
                     * Turn robot based on desired target 'x' coordinate location.
                     *----------------------------------------------------------------------------*/
                    driveRobot(linearPower, rotationalPower);

                    if( objectArea >= 60000)
                    {
                        stopRobot();
                        robotState = State.IDLE;
                    }
                }
                else if( rbVis.isTargetLocked() == false)
                {
                    stopRobot();
                }
            }
            case LOST_ROBOT:
            {

            }
            case IDLE:
            {

            }
        }/*End switch(robotState)*/

        /*------------------------------------------------------------------------------------------
         * Target locked, drive robot...
         *----------------------------------------------------------------------------------------*/
        if( rbVis.isTargetLocked() == true)
        {
            telemetry.addData("Position Error: ", xCoordinateSetpoint - objectXCoord);
            telemetry.addData("Motor power: ", rotationalPower);
            telemetry.addData("Coords:", "x: " + (int) objectXCoord + " y: " + (int) objectYCoord + " area: " + (int) (objectWidth * objectHeight));
            //telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
        }/*End if( rbVis.isTargetLocked() == true)*/

        telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
        telemetry.addData("Encoders",  "Right: " + rightCount + " Left: " + leftCount);

    }/*End public void loop()*/

    @Override
    public void stop()
    {
        super.stop();
        stopRobot();
    }

    private void driveRobot( double linearPower, double rotationalPower)
    {
        double rightPower = Range.clip(linearPower + rotationalPower, -1, 1);
        double leftPower = Range.clip(linearPower - rotationalPower, -1, 1);

        FrontmotorRight.setPower(rightPower);
        BackmotorRight.setPower(rightPower);
        FrontmotorLeft.setPower(leftPower);
        BackmotorLeft.setPower(leftPower);
    }

    private void resetPIDController()
    {
        prevError       = 0.0f;
        KiIntegral      = 0.0f;
        linearPower     = 0.0f;
        rotationalPower = 0.0f;
    }

    private double motorPIDController( double setPoint, double measurement)
    {
        double motorPower = 0.0f;
        double error = setPoint - measurement;
        double derivative = error - prevError;

        KiIntegral = KiIntegral + error*Ki;
        motorPower = KiIntegral + error*Kp + derivative*Kd;

        prevError = error;
        return motorPower;
    }

    private int getLeftEncoderCount()
    {
        int l_return = 0;

        if (FrontmotorLeft != null)
        {
            l_return = FrontmotorLeft.getCurrentPosition ();
        }

        return l_return;

    }

    private int getRightEncoderCount()

    {
        int l_return = 0;

        if (FrontmotorRight != null)
        {
            l_return = FrontmotorRight.getCurrentPosition ();
        }

        return l_return;

    } // a_right_encoder_count

    private void stopRobot()
    {
        linearPower     = 0.0f;
        rotationalPower = 0.0f;
        resetPIDController();
        driveRobot(linearPower, rotationalPower);
    }

    private void resetDriveEncoders()

    {
        if (FrontmotorRight != null)
        {
            FrontmotorRight.setMode( DcMotorController.RunMode.RESET_ENCODERS);
            FrontmotorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

        if (FrontmotorLeft != null)
        {
            FrontmotorLeft.setMode( DcMotorController.RunMode.RESET_ENCODERS);
            FrontmotorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }

    } // reset_drive_encoders
}
