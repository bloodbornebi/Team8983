/*
Copyright 2019 FIRST Tech Challenge Team 8983

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class TurnTest extends LinearOpMode {
    // Set up for encoder drive functions
    public ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // eg: TETRIX Motor Encoder
    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    
    private Gyroscope imu;
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private DcMotor arm;
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private Servo hook;
    private Servo sensor;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        LF = hardwareMap.get(DcMotor.class, "leftfront");
        RF = hardwareMap.get(DcMotor.class, "rightfront");
        LB = hardwareMap.get(DcMotor.class, "leftback");
        RB = hardwareMap.get(DcMotor.class, "rightback");
        arm = hardwareMap.get(DcMotor.class, "arm");
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        hook = hardwareMap.get(Servo.class, "hook");
        sensor = hardwareMap.get(Servo.class, "Sensor");
        
        LF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RF.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            leftTurn(.25);
            
            /* sleep(1000);
            encoderDrive(.5, 3, 3, 5);
            leftTurn(.5);
            sleep(1000);
            encoderDrive(.5, 3, 3, 5);
            rightTurn(.25);
            sleep(1000);
            encoderDrive(.5, 3, 3, 5);
            rightTurn(.5);
            */
            
            sleep(60000);
        }
    }
    
    public void leftTurn(double angle){
        double radius = 18 * 3.1415;
        encoderDrive(TURN_SPEED, radius*angle, 0, 5);
        sleep (1000);
    }
    
    public void rightTurn(double angle){
        double radius = 18 * 3.1415;
        encoderDrive(TURN_SPEED, 0, radius*angle, 5);
        sleep (1000);
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

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
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
           
            while (opModeIsActive() && (runtime.seconds() < timeout) && (LF.isBusy() || RF.isBusy()) && (LB.isBusy() || RB.isBusy() )) {
                telemetry.addData("Path1",  "Running to %7d :%7d", newLFTarget,  newRFTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",LF.getCurrentPosition(), RF.getCurrentPosition());
                telemetry.update();
            }

            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);
                
            sleep(250);   // optional pause after each move
        }
    }
}
