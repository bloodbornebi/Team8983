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

// DONE: grab platform

// DONE: rotate platform

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
@Autonomous(name="Platform", group="Linear Opmode")

public class SkystoneAutonomousPlatform extends LinearOpMode {
    // Set up for encoder drive functions
    public ElapsedTime runtime = new ElapsedTime();
    static final double GEAR = 1;
    static final double     COUNTS_PER_MOTOR_REV    = 288;    // eg: TETRIX Motor Encoder
    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.953;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = GEAR * (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = .5;
    static final double SLOW_SPEED = DRIVE_SPEED/5;
    static final double     TURN_SPEED              = DRIVE_SPEED/2;

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private Servo left;
    private Servo right;
    //private DcMotor arm;
    private DcMotor armend;

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "Left-Front");
        RF = hardwareMap.get(DcMotor.class, "Right-Front");
        LB = hardwareMap.get(DcMotor.class, "Left-Back");
        RB = hardwareMap.get(DcMotor.class, "Right-Back");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        //arm = hardwareMap.get(DcMotor.class, "arm");
        armend = hardwareMap.get(DcMotor.class, "armend");
        
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

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
        encoderDrive(DRIVE_SPEED,25,25,5);
        encoderDrive(SLOW_SPEED,5,5,30);

        // Grab platform
        // FIXME: these may be the wrong numbers tho
        left.setPosition(0.08);
        right.setPosition(.98);
        
        sleep(2500);

        // Turning to release platform
        encoderDrive(DRIVE_SPEED,-33,-33,5);
        
        // FIXME: these too may need to be changed
        left.setPosition(1);
        right.setPosition(0);

        // Park on tape
        encoderStrafe(DRIVE_SPEED,45,-1,5);
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

            int newLFTarget = LF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newRFTarget = RF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            int newLBTarget = LB.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int newRBTarget = RB.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            LF.setTargetPosition(newLFTarget);
            LB.setTargetPosition(newLBTarget);
            RF.setTargetPosition(newRFTarget);
            RB.setTargetPosition(newRBTarget);

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

            while (opModeIsActive() && (runtime.seconds() < timeout) && (LF.isBusy() || RF.isBusy() || LB.isBusy() || RB.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",LF.getCurrentPosition(), RF.getCurrentPosition());
                telemetry.update();
            }
            
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            sleep(250);   // optional pause after each move
        }
    }
    
    /**
    * @param speed max speed. 0 to 1
    * @param distance number of inches in the strafe direction
    * @param strafe direction to strafe, where -1 is left and 1 is right. non integer may have unexpected behavior
    * @param timeout seconds until timeout. for a low speed (<.5), should be fairly high (>30)
    */
    public void encoderStrafe(double speed, double distance, double strafe, double timeout) {
        if (opModeIsActive()) {
            resetEncoder();

            int newLFTarget = LF.getCurrentPosition() + (int)(distance * strafe * COUNTS_PER_INCH);
            int newRFTarget = RF.getCurrentPosition() + (int)(distance * strafe * -1 * COUNTS_PER_INCH);
            int newLBTarget = LB.getCurrentPosition() + (int)(distance * strafe * -1 * COUNTS_PER_INCH);
            int newRBTarget = RB.getCurrentPosition() + (int)(distance * strafe * COUNTS_PER_INCH);

            LF.setTargetPosition(newLFTarget);
            LB.setTargetPosition(newLBTarget);
            RF.setTargetPosition(newRFTarget);
            RB.setTargetPosition(newRBTarget);

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

            while (opModeIsActive() && (runtime.seconds() < timeout) && (LF.isBusy() || RF.isBusy() || LB.isBusy() || RB.isBusy())) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",LF.getCurrentPosition(), RF.getCurrentPosition());
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
