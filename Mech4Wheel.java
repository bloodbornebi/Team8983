
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
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="Mech4Drive", group="Linear Opmode")

public class Mech4Wheel extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor arm = null;
    private Servo lock;
    private Servo left;
    private Servo right;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF = hardwareMap.get(DcMotor.class, "Left-Front");
        RF = hardwareMap.get(DcMotor.class, "Right-Front");
        LB = hardwareMap.get(DcMotor.class,"Left-Back");
        RB = hardwareMap.get(DcMotor.class, "Right-Back");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lock = hardwareMap.get(Servo.class, "lock");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class,"right");
        double lockPos = 0;
        boolean rev = false;
        int stally = 0;

        // Most robots need the motor on one side to be reversed to drive forward - Reverse the motor that runs backwards when connected directly to the battery
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            //drive code
            double FORWARD = 1; //max
            double NEUTRAL = 0;
            double MOD = 0;
            double drive = (FORWARD - gamepad1.left_stick_y) - NEUTRAL;
            double strafe = gamepad1.left_stick_x - NEUTRAL;
            double rotate = gamepad1.right_stick_x - NEUTRAL;

            double lfPower = drive + strafe + rotate + MOD;
            double rfPower = drive - strafe - rotate + MOD;
            double lbPower = drive - strafe + rotate + MOD;
            double rbPower = drive + strafe - rotate + MOD;

            // Send calculated power to wheels
            LF.setPower(lfPower);
            RF.setPower(rfPower);
            LB.setPower(lbPower);
            RB.setPower(rbPower);

            // arm stuff
            if (gamepad1.left_trigger > .2) {
              arm.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > .2) {
              arm.setPower(-gamepad1.right_trigger);
            } else {
              arm.setPower(0);
            }

            // servo
            if (gamepad1.left_bumper) {
              lockPos = 0;
            } else if (gamepad1.right_bumper) {
              lockPos = 1;
            }
            lock.setPosition(lockPos);

            //robot reversals
            if (gamepad1.y) {
              LF.setDirection(DcMotor.Direction.FORWARD);
              RF.setDirection(DcMotor.Direction.REVERSE);
              LB.setDirection(DcMotor.Direction.FORWARD);
              RB.setDirection(DcMotor.Direction.REVERSE);
            } else if (gamepad1.b) {
              LF.setDirection(DcMotor.Direction.REVERSE);
              RF.setDirection(DcMotor.Direction.FORWARD);
              LB.setDirection(DcMotor.Direction.REVERSE);
              RB.setDirection(DcMotor.Direction.FORWARD);
            }

            //grabby Servo
            if (gamepad1.x) {
              left.setPosition(1);
              right.setPosition(1);
            } else if (gamepad1.a) {
              left.setPosition(0);
              right.setPosition(0);
            }

            //grabby Servo
            if (gamepad1.x) {
              left.setPosition(1);
              right.setPosition(1);
            } else if (gamepad1.a) {
              left.setPosition(0);
              right.setPosition(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftf (%.2f), rightf (%.2f)", lfPower, rfPower);
            telemetry.addData("Servos","pos "+lockPos);
            telemetry.addData("Switches",stally);
            telemetry.update();
        }
    }
}
