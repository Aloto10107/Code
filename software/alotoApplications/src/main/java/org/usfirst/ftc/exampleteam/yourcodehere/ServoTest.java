//* Copyright (c) 2014 Qualcomm Technologies Inc

//All rights reserved.

//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:

//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.

//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.

//Neither the name of Qualcomm Technologies Inc nor the names of its contributors
//may be used to endorse or promote products derived from this software without
//specific prior written permission.

//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;

import java.util.ArrayList;
import java.util.List;
//

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ServoTest extends OpMode {

   /*
    * Note:Initalizing motors, right and left motors will have same values. The third wheel will have
    * the same values as the left side if we use it.
    */



    DcMotor FrontmotorRight;
    DcMotor FrontmotorLeft;
    DcMotor BackmotorRight;
    DcMotor BackmotorLeft;
    DcMotor Arm;
    DcMotor Hook;
    DcMotor Winch;
    Servo Sweeper;
    Servo Sweepers[];
    List<Servo> servoList = new ArrayList<Servo>();


    int armDutyCycle;
    int armDutyCount;
    int hookDutyCycle;
    int hookDutyCount;

    //Servo ServoLid;
    //ServoController sc;
    //DcMotor ArmRight;

	/*
	 *Note: Things that are in purple can be uncommented when a motor is needed for that action!
	 */
    //DcMotor Sweeper;


    /**
     * Constructor
     */
    public ServoTest() {

    }

    /*
        * Code to run when the op mode is first enabled goes here (This is the Op Mode you use on
        * the DRIVER STATION.
        *
        * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
        */
    @Override
    public void init() {



      /*
       * Use the hardwareMap to get the dc motors by name. Note
       * that the names of the devices must match the names used when you
       * configure your robot and created the configuration file.
       */
      
      /*
       * For the Duggan bot we assume the following,
       *   There are 6 motors "FrontmotorRight", "FrontmotorLeft", "BackmotorRight", "BackmotorLeft"
       *   and "TheThirdWheel, "Servo ServoSwiper", "ServoController sc", and "ArmRight"

       *   "FrontmotorRight" is on the right side of the bot in the front. (Driver 1)
       *   "FrontmotorLeft" is on the left side of the bot and reversed, in the front.(Driver 1)
       *   "BackmotorRight" is on the right side of the bot in the back.(Driver 1)
       *   "BackmotorLeft" is on the left side of the bot and reversed, in the back.(Driver 1)
       *   "Lift" is in the front of the robot and control the lower part of the robot.
       *   (Driver 2) Allows lower arm to move in an up and down motion.
       *   "Hook" for the motion of the top of the arm, moving up and down, and can
       *   follow the left side of the bot.
       *   "Servo ServoSwiper" is
       *   "BackmotorRIght" is
       *   "ArmRight" is
       */
        FrontmotorRight = hardwareMap.dcMotor.get("motor_fr");
        FrontmotorLeft = hardwareMap.dcMotor.get("motor_fl");
        BackmotorRight = hardwareMap.dcMotor.get("motor_br");
        BackmotorLeft = hardwareMap.dcMotor.get("motor_bl");
        Arm = hardwareMap.dcMotor.get("Arm");
        Hook = hardwareMap.dcMotor.get("Hook");
        Winch = hardwareMap.dcMotor.get("winch");
        Sweeper = hardwareMap.servo.get("sweeper");
        servoList.add(Sweeper);

        FrontmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BackmotorLeft.setDirection(DcMotor.Direction.REVERSE);
        BackmotorRight.setDirection(DcMotor.Direction.REVERSE);

        ClassFactory.createEasyMotorController(this, FrontmotorLeft, BackmotorLeft);
        ClassFactory.createEasyMotorController(this, FrontmotorRight, BackmotorRight);
        ClassFactory.createEasyServoController(this, servoList);
        resetDriveEncoders();
        FrontmotorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FrontmotorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        Sweepers[1] = Sweeper;

        armDutyCycle = 1;
        hookDutyCycle = 1;
        armDutyCount = 0;
        hookDutyCount = 0;
    }

    int getLeftEncoderCount() {
        int l_return = 0;

        if (FrontmotorLeft != null) {
            l_return = FrontmotorLeft.getCurrentPosition();
        }

        return l_return;

    }

    int getRightEncoderCount()

    {
        int l_return = 0;

        if (FrontmotorRight != null) {
            l_return = FrontmotorRight.getCurrentPosition();
        }

        return l_return;

    } // a_right_encoder_count

    public void resetDriveEncoders()

    {
        if (FrontmotorRight != null) {
            FrontmotorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }

        if (FrontmotorLeft != null) {
            FrontmotorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    } // reset_drive_encoders

    /*
        * This method will be called repeatedly in a loop
        *
        * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
        */
    @Override
    public void loop() {

      /*
       * Gamepad `
       * 
       * Gamepad 1 controls the motors via the left and right stick,
       * Gamepad 2 (Currently not in use) controls the sweeper
       */

        // throttleRight: right_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
        // throttleLeft: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        float throttleLeft = -gamepad1.left_stick_y;
        float throttleRight = -gamepad1.right_stick_y;
        float throttleHook = -gamepad2.right_stick_y;
        float throttleArm = -gamepad2.left_stick_y;
        float throttleWinch = -gamepad2.left_stick_y;

        float right = throttleRight;
        float left = throttleLeft;
        float thisaffectsthearm = throttleArm;
        float thisaffectsthehook = throttleHook;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        right = (float) scaleInput(right, 1);
        left = (float) scaleInput(left, 1);

        thisaffectsthearm = (float) scaleInput(thisaffectsthearm, 1);
        thisaffectsthehook = (float) scaleInput(thisaffectsthehook, 1);


        double servoPosition = 0;
        if (this.gamepad2.dpad_up)
        {
            servoPosition = servoPosition + 0.1;
            if (servoPosition > 1) servoPosition = 1;
        }
        if (this.gamepad2.dpad_down)
        {
            servoPosition = servoPosition - 0.1;
            if (servoPosition < 0) servoPosition = 0;
        }
        Sweeper.setPosition(servoPosition);


/*        double servoPosition = 0.1;
        if (servoPosition < 0.9)
        {
            servoPosition = servoPosition + 0.1;
        }
        else
        {
            servoPosition = servoPosition - 0.1;
        }
        Sweeper.setPosition(servoPosition);*/


        FrontmotorRight.setPower(right);
        BackmotorRight.setPower(right);
        FrontmotorLeft.setPower(left);
        BackmotorLeft.setPower(left);
        //Sweeper.setPower(bothsweeper);

        Hook.setPower(thisaffectsthehook);
        Arm.setPower(thisaffectsthearm);
        Winch.setPower(thisaffectsthearm);


        //int leftCount = getLeftEncoderCount();
        //int rightCount = getRightEncoderCount();



      /*
       * Send telemetry data back to driver station. Note that if we are using
       * a legacy NXT-compatible motor controller, then the getPower() method
       * will return a null value. The legacy NXT-compatible motor controllers
       * are currently write only.
       */
        telemetry.addData("Text", "*** Duggan Data***");
        //telemetry.addData("Encoders",  "Right: " + rightCount + " Left: " + leftCount);
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
        //telemetry.addData("sweeper tgt pwr","bothsweeper: " + String.format("%.2f", bothsweeper));


    }

    /*

        * Code to run when the op mode is first disabled goes here
        *
        * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
        */
    @Override
    public void stop() {

    }


	/*
        * This method scales the joystick input so for low joystick values, the
        * scaled value is less than linear.  This is to make it easier to drive
        * the robot more precisely at slower speeds.
        */

    double scaleInput(double dVal, double scaleFactor) {
        //double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
        //		0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };
        double[] scaleArray = {0.0, 0.011, 0.013, 0.0266, 0.0491, 0.0814, 0.123, 0.1754,
                0.2371, 0.308, 0.38, 0.48, 0.58, 0.69, 0.81, .94, 1.00};
        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            if (index < 16)
                dScale = -scaleArray[index] / scaleFactor;
            else
                dScale = -scaleArray[index];
        } else {
            if (index < 16)
                dScale = scaleArray[index] / scaleFactor;
            else
                dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
		/*
        * This method scales the joystick input so for low joystick values, the
        * scaled value is less than linear.  This is to make it easier to drive
        * the robot more precisely at slower speeds.
        */


    double scaleArmHook(double dVal, double scaleFactor) {
        double[] scaleArray = {0.0, 0.025, 0.045, 0.05, 0.06, 0.075, 0.09, 0.12,
                0.15, 0.18, 0.215, 0.25, 0.30, 0.36, 0.425, .5, .5};


        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            if (index < 16)
                dScale = -scaleArray[index] / scaleFactor;
            else
                dScale = -scaleArray[index];
        } else {
            if (index < 16)
                dScale = scaleArray[index] / scaleFactor;
            else
                dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}