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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.Console;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class Hardware_FTC_OpMode extends OpMode
{
    /* Public OpMode members. */
    public DcMotor leftDrive      = null;
    public DcMotor rightDrive     = null;
    public DcMotor arm            = null;
    public Servo   claw           = null;
    public DcMotor extend         = null;
    public DcMotor leftDriveBack  = null;
    public DcMotor rightDriveBack = null;

    public final static double CLAW_MIN_RANGE   = 0.2;
    public final static double CLAW_MAX_RANGE   = 0.7;
    public final static double CLAW_HOME        = 0.2;

    /* Local OpMode members. */
    private HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // These must be overloaded to avoid build errors for some reason

    public void loop() { }
    public void init() { }

    /* Constructor */
    public void Hardware_FTC_OpMode() { }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        arm            = hwMap.get(DcMotor.class, "arm");
        claw           = hwMap.get(Servo.class, "claw");
        extend         = hwMap.get(DcMotor.class, "extend");
        leftDrive      = hwMap.get(DcMotor.class, "left_drive");
        rightDrive     = hwMap.get(DcMotor.class, "right_drive");
        leftDriveBack  = hwMap.get(DcMotor.class, "left_drive_back");
        rightDriveBack = hwMap.get(DcMotor.class, "right_drive_back");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        leftDriveBack.setPower(0);
        rightDrive.setPower(0);
        rightDriveBack.setPower(0);
        arm.setPower(0);
        claw.setPosition(CLAW_HOME);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    public void Auto() {
        //Do things for our amusement
        /* Things it must du:
         * Scan QR Code and place block in boi
         * Push ball of opposite color off of platboi
         * balance on platboi
         */
        
    }

    // functions we probs need for autonomous

    public int scanPictorgraph() {
        // Do something to scan the pictograph(s)
        return 0;
    }
    public int getPictographType() {
        // Return 1 if picture is left
        // Return 2 if picture is center
        // Return 3 if picture is right
        return 0;
    }
    public int scanColor() {
        // Do something to scan color
        return 0;
    }
    public int getColor() {
        // Return 1 if yellow
        // Return 2 if black
        return 0;
    }
    public void rebalanceRobot() {
        // Do something to rebalance the robot so
        // it can safely stand on the balance platform
    }
    public void doStuffWithBlocks() {
        // Self explanatory
    }
}

// Reference for controller:
// https://images-na.ssl-images-amazon.com/images/I/91RsGVBf1IL._SL1500_.jpg

/*
Steps to fix the no usb devices recognized shit:
1) Turn off instant run (file > settings > build, execution. deployment > instant run)
2) Set deployment target option to USB device (run > android app > project > general > deployment target options > usb device)
3) Plug DATA TRANSFER cable into phone/compooter, set usb transfer to MTP, run selected configuration
 */
