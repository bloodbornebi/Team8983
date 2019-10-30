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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
@Autonomous(name="EncoderTest", group="Linear Opmode")

public class EncoderTest extends LinearOpMode {
    // Set up for encoder drive functions
    public ElapsedTime runtime = new ElapsedTime();
    static final double GEAR = 1.5;
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = GEAR * (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotorEx left;
    private DcMotorEx right;
    private DcMotorEx arm;
    private Servo lock;

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotorEx.class, "Left");
        right = hardwareMap.get(DcMotorEx.class, "Right");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        lock = hardwareMap.get(Servo.class,"lock");

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
        double test = 1;
        encoderDrive(DRIVE_SPEED,test,test,2);
    }

    /**
     * @param angle Degree to turn. 360=1.
     */
    public void leftTurn(double angle){
      telemetry.addData("Turn", "Turning "+(angle*360)+"º to the left");
      telemetry.update();
      double radius = 14;
      double circum = 14 * 2 * 3.1415;
      encoderDrive(TURN_SPEED, radius*angle, 0, 5);
      left.setPower(0);
      right.setPower(0);
      sleep (250);
    }

    public void rightTurn(double angle){
      telemetry.addData("Turn", "Turning "+(angle*360)+"º to the right");
      telemetry.update();
      double radius = 14;
      double circum = 14 * 2 * 3.1415;
      encoderDrive(TURN_SPEED, 0, circum*angle, 5);
      left.setPower(0);
      right.setPower(0);
      sleep (250);
    }

    public void resetEncoder(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            resetEncoder();

            newLeftTarget = left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);

            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left.setVelocity(-200);
            right.setVelocity(200);
            
            while(left.isBusy()) {
                // Let the drive team see that we're waiting on the motor
                telemetry.addData("Status", "Waiting for the motor to reach its target");
                telemetry.update();
            }

            
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
