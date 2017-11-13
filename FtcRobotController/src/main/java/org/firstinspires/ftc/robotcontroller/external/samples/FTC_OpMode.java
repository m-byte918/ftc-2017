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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FTC_OpMode: TeleOp Tank", group="TeleOp")
@Disabled
public class FTC_OpMode extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_FTC_OpMode robot = new Hardware_FTC_OpMode();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double clawSpeed = 0.5;
        double clawPosition = robot.CLAW_HOME
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left  = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            robot.leftDrive.setPower(left);
            robot.leftDriveBack.setPower(left);
            robot.rightDrive.setPower(right);
            robot.rightDriveBack.setPower(right);
            
            // moves arm up and down and in and out 
            int  up_out  = 10; 
            int  down_in =-10; 
            
            if (gamepad1.a) {
                robot.extend.setPower(up_out); 
            } else {
                robot.extend.setPower(0);
            }
            
            if (gamepad1.b) {
                robot.extend.setPower(down_in); 
            } else {
                robot.extend.setPower(0);
            }
        
            if (gamepad1.dpad.up) {
                robot.arm.setPower(up_out);
            } else {
                robot.arm.setPower(0); 
            }
            
            if ( gamepad1.dpad.down) {
                robot.arm.setPower(down_in);
            } else { 
                robot.arm.setPower(0);
            }
            
            // changes position of claw servo 
            if (gamepad1.x) {
                clawPosition += clawSpeed; 
            } 
            if (gamepad1.y) {
                clawPosition -= clawSpeed;
            } 
            
            // sets new position of claw servo 
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);

            // Send telemetry message to signify robot running;
            telemetry.addLine("Motors and Servo ~")
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("arm", "%.2f", up_out, down_in);
            telemetry.addData("extend", "%.2f", up_out, down_in); 
            telemetry.addLine("left joystick ~");
            telemetry.addData("y1", gamepad1.left_stick_y);
            telemetry.addLine("right joystick ~");
            telemetry.addData("y2", gamepad1.right_stick_y);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
            // Don't fucking change that xd
        }
    }
}