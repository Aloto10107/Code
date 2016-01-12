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

import com.qualcomm.ftcrobotcontroller.opmodes.VisionOpMode;

import org.opencv.core.Rect;

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

    @Override
    public void init() {
        super.init();

        //this.setCamera(Cameras.PRIMARY);
        //this.setFrameSize(new Size(400, 400));

    }

    @Override
    public void loop() {
        super.loop();
        if( rbVis.isTargetLocked() == true)
        {
            int[] rawTarget = rbVis.getRawTargetCoords();
            double[] filteredTarget = rbVis.getFilteredTargetCoords();
            objectXCoord     = filteredTarget[0];
            objectYCoord     = filteredTarget[1];
            objectdx    = filteredTarget[2];
            objectdy     = filteredTarget[3];
            objectWidth  = filteredTarget[4];
            objectHeight = filteredTarget[5];

            telemetry.addData("Coords:", "x: " + (int)objectXCoord + " y: " + (int)objectYCoord + " area: " + (int)(objectWidth*objectHeight));
        }

        telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
        telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
    }

    @Override
    public void stop() {
        super.stop();
    }

    private double motorControlPID( double error)
    {
        double motorPower = 0.0f;
        return motorPower;
    }
}
