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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FTC_OpMode: TeleOp Tank", group="TeleOp")
@Disabled
public class FTC_OpMode extends OpMode {

    /* Declare OpMode members. */
    Hardware_FTC_OpMode robot = new Hardware_FTC_OpMode();

    double left;
    double right;
    //double clawSpeed = 3;
    //private double clawPosition = robot.CLAW_HOME;

    // Initialize the hardware variables.
    // The init() method of the hardware class does all the work here
    public void init(){
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
    }

    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left  = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        // Move wheels
        robot.leftDrive.setPower(left);
        robot.leftDriveBack.setPower(left);
        robot.rightDrive.setPower(right);
        robot.rightDriveBack.setPower(right);

        // Open/close claw
        if (gamepad1.a && gamepad1.b) {
            robot.claw.setPower(0);
        } else {
            if (gamepad1.a)
                robot.claw.setPower(-10);
            else if (gamepad1.b)
                robot.claw.setPower(10);
        }

        // Move elevator motors up/down
        if (gamepad1.dpad_up && gamepad1.dpad_down) {
            robot.leftElevator.setPower(0);
            robot.rightElevator.setPower(0);
        } else {
            if (gamepad1.dpad_up) {
                robot.leftElevator.setPower(50);
                robot.rightElevator.setPower(50);
            }
            else if (gamepad1.dpad_down) {
                robot.leftElevator.setPower(-50);
                robot.rightElevator.setPower(-50);
            }
        }

        // Send telemetry message to signify robot running;
        telemetry.addLine("Motors ~");
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.update();
    }
}

/*
Controls (for now):

Joysticks:  Move wheels
Button A:   Close claw
Button B:   Open claw
Dpad-Up:    Move elevator/claw up
Dpad-Down:  Move elevator/claw down
Dpad-Left:  Move sideways left (move middle wheel left)
Dpad-Right: Move sideways right (move middle wheel right)

Controller reference: https://images-na.ssl-images-amazon.com/images/I/91RsGVBf1IL._SL1500_.jpg
*/
