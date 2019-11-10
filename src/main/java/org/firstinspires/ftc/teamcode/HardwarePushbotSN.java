/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwarePushbotSN
{
    /* Public OpMode members. */
    public DcMotor  CH_M_FL   = null;
    public DcMotor  CH_M_FR  = null;
    public DcMotor  CH_M_BL   = null;
    public DcMotor  CH_M_BR  = null;
    public DcMotor  LS_M_SL  = null;
    public DcMotor  LS_M_TR     = null;

    Servo FD_S_1;
    Servo FD_S_2;
    Servo GR_S_1;
    Servo GR_S_2;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbotSN(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        CH_M_FL  = hwMap.get(DcMotor.class, "CH_M_FL");
        CH_M_FR  = hwMap.get(DcMotor.class, "CH_M_FR");
        CH_M_BL  = hwMap.get(DcMotor.class, "CH_M_BL");
        CH_M_BR  = hwMap.get(DcMotor.class, "CH_M_BR");
        LS_M_SL  = hwMap.get(DcMotor.class, "LS_M_SL");
        LS_M_TR  = hwMap.get(DcMotor.class, "LS_M_TR");

        FD_S_1 = hwMap.servo.get("FD_S_1");
        FD_S_2 = hwMap.servo.get("FD_S_2");
        GR_S_1 = hwMap.servo.get("GR_S_1");
        GR_S_2 = hwMap.servo.get("GR_S_2");

        CH_M_FL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        CH_M_BL.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        CH_M_FR.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        CH_M_BR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        CH_M_FL.setPower(0);
        CH_M_FR.setPower(0);
        CH_M_BL.setPower(0);
        CH_M_BR.setPower(0);
        LS_M_SL.setPower(0);
        LS_M_TR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        CH_M_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CH_M_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CH_M_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CH_M_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LS_M_SL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LS_M_TR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

    }
    public void prepEncoderDrive(){
        CH_M_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CH_M_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CH_M_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CH_M_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LS_M_SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LS_M_TR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        CH_M_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CH_M_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CH_M_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CH_M_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LS_M_SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LS_M_TR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}



