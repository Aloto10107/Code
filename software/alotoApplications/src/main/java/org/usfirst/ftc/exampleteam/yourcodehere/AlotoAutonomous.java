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
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Rect;
import org.swerverobotics.library.ClassFactory;

import java.util.ArrayList;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class AlotoAutonomous extends VisionOpMode {

    private double objectXCoord;
    private double objectYCoord;
    private double objectdx;
    private double objectdy;
    private double objectWidth;
    private double objectHeight;
    private double Kp;
    private double Ki;
    private double Kd;
    private double KiIntegral;
    private double rotationalPower;
    private double linearPower;
    private double xCoordinateSetpoint;
    private double yCoordinateSetpoint;
    private double prevError;

    DcMotor FrontmotorRight;
    DcMotor FrontmotorLeft;
    DcMotor BackmotorRight;
    DcMotor BackmotorLeft;

    @Override
    public void init() {
        super.init();
        FrontmotorRight   = hardwareMap.dcMotor.get("motor_fr");
        FrontmotorLeft    = hardwareMap.dcMotor.get("motor_fl");
        BackmotorRight    = hardwareMap.dcMotor.get("motor_br");
        BackmotorLeft     = hardwareMap.dcMotor.get("motor_bl");

        FrontmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BackmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BackmotorRight.setDirection(DcMotor.Direction.REVERSE);

        ClassFactory.createEasyMotorController(this, FrontmotorLeft, BackmotorLeft);
        ClassFactory.createEasyMotorController(this, FrontmotorRight, BackmotorRight);

        //this.setCamera(Cameras.PRIMARY);
        //this.setFrameSize(new Size(400, 400));


        Ki = .0001f;
        Kp = .001f;
        Kd = 0.0f;

        xCoordinateSetpoint = 0.0f;
        yCoordinateSetpoint = 0.0f;
        resetPIDController();
    }

    @Override
    public void loop()
    {
        super.loop();

        //float Ki = -gamepad1.left_stick_y/100;

        //if( Ki < 0.0f)
        //    Ki = 0.0f;
        
        /*------------------------------------------------------------------------------------------
         * Target locked, drive robot...
         *----------------------------------------------------------------------------------------*/
        if( rbVis.isTargetLocked() == true)
        {
            int[] rawTarget         = rbVis.getRawTargetCoords();
            double[] filteredTarget = rbVis.getFilteredTargetCoords();
            objectXCoord            = filteredTarget[0]; /*Target 'x' position on camera screen*/
            objectYCoord            = filteredTarget[1]; /*Target 'y' position on camera screen*/
            objectdx                = filteredTarget[2]; /*The speed of the target on the x plane (pixels/sec)*/
            objectdy                = filteredTarget[3]; /*The speed of the target on the y plane (pixels/sec)*/
            objectWidth             = filteredTarget[4]; /*The width of the target as seen by the camera*/
            objectHeight            = filteredTarget[5]; /*The height of the target as seen by the camera*/

            /*--------------------------------------------------------------------------------------
             * Smooth out the error in the 'x' position before using it to turn the robot left or
             * right.
             *------------------------------------------------------------------------------------*/
            rotationalPower = motorPIDController(xCoordinateSetpoint, objectXCoord);

            /*--------------------------------------------------------------------------------------
             * Turn robot based on desired target 'x' coordinate location.
             *------------------------------------------------------------------------------------*/
            driveRobot(linearPower, rotationalPower);

            telemetry.addData("Position Error: ", xCoordinateSetpoint - objectXCoord);
            telemetry.addData("Motor power: ", rotationalPower);
            telemetry.addData("Coords:", "x: " + (int) objectXCoord + " y: " + (int) objectYCoord + " area: " + (int) (objectWidth * objectHeight));
        }/*End if( rbVis.isTargetLocked() == true)*/
        else
        {
            /*--------------------------------------------------------------------------------------
             * Target lost, stop robot...
             *------------------------------------------------------------------------------------*/
            driveRobot(0,0);
            resetPIDController();

        }

        telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
        telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
        telemetry.addData("Kp: ", Kp);
        telemetry.addData("Ki: ", Ki);
        telemetry.addData("Kd: ", Kd);
        telemetry.addData("Integral Error: ", KiIntegral);

    }

    @Override
    public void stop()
    {
        super.stop();
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
}
