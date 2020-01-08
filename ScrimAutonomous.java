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

// DONE: drive to platform
  // stretch: separate out encoder methods

// TODO: grab platform
  // reach out arm
  // clasp
  // pull closer

// TODO: rotate platform
  // spin to right position
  // push platform
  // release grip

// TODO: park on tape
  // orient to walls
  // drive forward (encoder methods)

// TODO: stretch: grab block
  // drive to line
  // scan blocks
  // pick up block
  // drive to platform

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="ScrimAutonomousRed", group="Linear Opmode")

public class ScrimAutonomous extends LinearOpMode {
    // Set up for encoder drive functions
    public ElapsedTime runtime = new ElapsedTime();
    static final double GEAR = 1.5;
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = GEAR * (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private Servo left;
    private Servo right;

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "Left-Front");
        RF = hardwareMap.get(DcMotor.class, "Right-Front");
        LB = hardwareMap.get(DcMotor.class, "Left-Back");
        RB = hardwareMap.get(DcMotor.class, "Right-Back");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        left.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        right.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        telemetry.addData("Status", "Running");
        telemetry.update();

        resetEncoder();

        /* End of set up code, real code begins */

        // Driving to platform
        encoderDrive(DRIVE_SPEED,47.25,45.25,5); //This goes exactly to the platform - it may be necessary to not go quite so far
    }

    /**
     * @param angle Degree to turn. 360=1.
     */
    public void leftTurn(double angle){
      double radius = 14;
      double circum = 14 * 2 * 3.1415;
      encoderDrive(TURN_SPEED, radius*angle, 0, 5);
      LF.setPower(0);
      LB.setPower(0);
      RF.setPower(0);
      RB.setPower(0);
      sleep (250);
    }

    public void rightTurn(double angle){
      double radius = 14;
      double circum = 14 * 2 * 3.1415;
      encoderDrive(TURN_SPEED, 0, circum*angle, 5);
      LF.setPower(0);
      LB.setPower(0);
      RF.setPower(0);
      RB.setPower(0);
      sleep (250);
    }

    public void resetEncoder(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            resetEncoder();

            newLeftTarget = LF.getCurrentPosition() +
            LB.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RF.getCurrentPosition() +
            RB.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            LF.setTargetPosition(newLeftTarget);
            LB.setTargetPosition(newLeftTarget);
            RF.setTargetPosition(newRightTarget);
            RB.setTargetPosition(newRightTarget);

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LF.setPower(Math.abs(speed));
            LB.setPower(Math.abs(speed));
            RF.setPower(Math.abs(speed));
            RB.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeout) && (left.isBusy() || right.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",left.getCurrentPosition(), right.getCurrentPosition());
                telemetry.update();
            }

            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
