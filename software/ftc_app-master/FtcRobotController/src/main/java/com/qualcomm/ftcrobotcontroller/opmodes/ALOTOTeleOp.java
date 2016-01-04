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

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;
//

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class ALOTOTeleOp extends OpMode {

   /*
    * Note:Initalizing motors, right and left motors will have same values. The third wheel will have
    * the same values as the left side.
    */

	DcMotor FrontmotorRight;
	DcMotor FrontmotorLeft;
	DcMotor BackmotorRight;
	DcMotor BackmotorLeft;
	DcMotor TheThirdWheel;
	DcMotor TheThirdWheelLift;
	Servo ServoSwiper;
	ServoController sc;
	//DcMotor ArmRight;

	/*
	 *Note: Things that aren't in purple can be commented out when a motor is needed for that action!
	 */
	//DcMotor Sweeper;


	/**
	 * Constructor
	 */
	public ALOTOTeleOp() {

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
       * configured your robot and created the configuration file.
       */
      
      /*
       * For the Duggan bot we assume the following,
       *   There are 8 motors "FrontmotorRight", "FrontmotorLeft", "BackmotorRight", "BackmotorLeft"
       *   and "TheThirdWheel, "Servo ServoSwiper", "ServoController sc", and "ArmRight"

       *   "FrontmotorRight" is on the right side of the bot in the front.
       *   "FrontmotorLeft" is on the left side of the bot and reversed, in the front.
       *   "BackmotorRight" is on the right side of the bot in the back.
       *   "BackmotorLeft" is on the left side of the bot and reversed, in the back.
       *   "TheThirdWheel" is in the front of the robot and is reversed. To make the front wheel
       *   follow the left side of the bot.
       *   "Servo ServoSwiper" is
       *   "BackmotorRIght" is
       *   "ArmRight" is
       */
		FrontmotorRight   = hardwareMap.dcMotor.get("motor_fr");
		FrontmotorLeft    = hardwareMap.dcMotor.get("motor_fl");
		BackmotorRight    = hardwareMap.dcMotor.get("motor_br");
		BackmotorLeft     = hardwareMap.dcMotor.get("motor_bl");
		TheThirdWheel     = hardwareMap.dcMotor.get("third_wheel");
		TheThirdWheelLift = hardwareMap.dcMotor.get("third_wheel_lift");
        //ArmRight = hardwareMap.dcMotor.get("arm");
		//sc = hardwareMap.servoController.get("matrixServo");
		//sc.pwmEnable();
		//ServoSwiper = hardwareMap.servo.get("swiper");

		FrontmotorLeft.setDirection(DcMotor.Direction.REVERSE);
		BackmotorLeft.setDirection(DcMotor.Direction.REVERSE);
		BackmotorRight.setDirection(DcMotor.Direction.REVERSE);
		//FrontmotorRight.setDirection(DcMotor.Direction.REVERSE);
		//TheThirdWheel.setDirection(DcMotor.Direction.REVERSE);


		//Sweeper = hardwareMap.dcMotor.get("motor_sweeper");

	}
	/*
        * This method will be called repeatedly in a loop
        *
        * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
        */
	@Override
	public void loop() {

      /*
       * Gamepad 1
       * 
       * Gamepad 1 controls the motors via the left and right stick,
       * Gamepad 2 (Currently not in use) controls the sweeper
       */

		// throttleRight: right_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
		// throttleLeft: left_stick_y ranges from -1 to 1, where -1 is full up, and
		// 1 is full down
		float throttleLeft     = -gamepad1.left_stick_y;
		float throttleRight    = -gamepad1.right_stick_y;
		float battleAxThrottle = -gamepad2.right_stick_y;
		float battleAxLift     = -gamepad2.left_stick_y;

		float right = throttleRight;
		float left = throttleLeft;

		//float triggerrightup = -gamepad2.right_trigger;
		//float triggerleftdown = -gamepad2.left_trigger;
		//float bothsweeper = triggerrightup - triggerleftdown;


		// clip the right/left values so that the values never exceed +/- 1
		right = Range.clip(right, -1, 1);
		left = Range.clip(left, -1, 1);

		battleAxThrottle = Range.clip(battleAxThrottle, -1, 1);
		battleAxLift     = Range.clip(battleAxLift, -1, 1);

		//Lift = Range.clip(right, -1, 1);
		//bothsweeper = Range.clip(bothsweeper, -1, 1);

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		right            = (float)scaleInput(right, 1);
		left             = (float)scaleInput(left, 1);
		battleAxThrottle = (float)scaleInput(battleAxThrottle, 1);
		battleAxLift     = (float)scaleInput(battleAxLift, 2);

		//Lift = (float)scaleInput(right);
		//bothsweeper = (float)scaleInput(bothsweeper);
		//ServoSwiper.setPosition(1);
		// write the values to the motor
		FrontmotorRight.setPower(right);
		BackmotorRight.setPower(right);
		FrontmotorLeft.setPower(left);
		BackmotorLeft.setPower(left);
		//Sweeper.setPower(bothsweeper);
		TheThirdWheel.setPower(battleAxThrottle);
		TheThirdWheelLift.setPower(battleAxLift);
		//ArmRight.setPower(Lift);


      /*
       * Send telemetry data back to driver station. Note that if we are using
       * a legacy NXT-compatible motor controller, then the getPower() method
       * will return a null value. The legacy NXT-compatible motor controllers
       * are currently write only.
       */
		telemetry.addData("Text", "*** Duggan Data***");
		telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
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

	double scaleInput(double dVal, double scaleFactor)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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
			dScale = -scaleArray[index] / scaleFactor;
		} else {
			dScale = scaleArray[index] / scaleFactor;
		}

		// return scaled value.
		return dScale;
	}

}