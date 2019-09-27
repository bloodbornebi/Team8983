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

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Map;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutonomousCrater", group="Pushbot")

public class AutonomousCrater extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
   // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private DcMotor gear;
    private DcMotor frame;
    private DcMotor armDown;
    private Servo Sensor;
    private Servo Pivot;
    ColorSensor Color;
    
    int red;
  
    boolean left;
    
    @Override
    public void runOpMode() throws InterruptedException {
        LF  = hardwareMap.get(DcMotor.class, "leftfront");
        RF = hardwareMap.get(DcMotor.class, "rightfront");
        LB  = hardwareMap.get(DcMotor.class, "leftback");
        RB = hardwareMap.get(DcMotor.class, "rightback");
        Color=hardwareMap.get(ColorSensor.class, "Color");
        Sensor=hardwareMap.get(Servo.class, "Sensor");
        LF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RF.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        gear = hardwareMap.get(DcMotor.class, "gear");
        frame = hardwareMap.get(DcMotor.class, "frame");
        armDown = hardwareMap.get(DcMotor.class, "arm");
        waitForStart();

        resetStartTime();
        int blueLeft;
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                         LF.getCurrentPosition(),
                         RF.getCurrentPosition(),
                         LB.getCurrentPosition(),
                         RB.getCurrentPosition());
        telemetry.update();
        resetEncoder();
          
        resetStartTime();
     
        /* DRIVE PATH (Approximation)
            - Drop off the box
            - Drive straight
            - Drop the thing
        */
        
        //moves to ball
        encoderDrive(.3,  5,  5,  5);
        sleep(1000);
        pushBall();
        turn();
        pushBall();
        turn();
        pushBall();
        turn();
        sleep(1000);
        encoderDrive(.3,6,0, 4);
        sleep(1000);
        encoderDrive(.3,10,10, 4);
        sleep(2200);
        
        telemetry.addData("Path", "Complete");
        telemetry.update();        
    }
    
    
    
    public void pushBall() {
        int blue= Color.blue();
        telemetry.addData("Color", blue);
        telemetry.update();
        if (blue<20) {
            encoderDrive(.3,  4,  4,  5.0);
            encoderDrive(.3,  -4,  -4,  5.0);
        }
    }
    
    public void turn(){
        encoderDrive(.3, -4, -4, 5);
        sleep(1000);
        encoderDrive(.3, -6, 6, 5);
        sleep(1000);  
        encoderDrive(.3, 10, 10, 5);
        sleep(1000);
        encoderDrive(.3, 6, 0, 5);
        sleep(1000);
        encoderDrive(.3, 4, 4, 5); 
    }
    
    public void resetEncoder(){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeout) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
       
        if (opModeIsActive()) {
            newLFTarget = LF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRFTarget = RF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLBTarget = LB.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRBTarget = RB.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        
            LF.setTargetPosition(newLFTarget);
            RF.setTargetPosition(newRFTarget);
            LB.setTargetPosition(newLBTarget);
            RB.setTargetPosition(newRBTarget);
           

            LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
            // reset the timeout time and start motion.
            runtime.reset();
            LF.setPower(Math.abs(speed));
            RF.setPower(Math.abs(speed));
            LB.setPower(Math.abs(speed));
            RB.setPower(Math.abs(speed));
           
            while (opModeIsActive() &&
                   (runtime.seconds() < timeout) &&
                   (LF.isBusy() || RF.isBusy()) && (LB.isBusy() || RB.isBusy() )) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",LF.getCurrentPosition(), RF.getCurrentPosition());
                                        
                telemetry.update();
            }

           
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
            sleep(250);   // optional pause after each move
        }
    }
}
