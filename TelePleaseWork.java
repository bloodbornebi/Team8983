// Waiting for official hardware set up to finish work - everything labeled (?)

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
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name="linearAop", group="Linear Opmode")

public class TelePleaseWork extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LB;
    private DcMotor RB;
    private DcMotor armDown;
    private Servo hook;
    private Servo marker;
 
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // TODO: Add other motors, etc
        LF = hardwareMap.get(DcMotor.class, "leftfront");
        RF = hardwareMap.get(DcMotor.class, "rightfront");
        LB = hardwareMap.get(DcMotor.class, "leftback");
        RB = hardwareMap.get(DcMotor.class, "rightback");
        armDown = hardwareMap.get(DcMotor.class, "arm");
        hook = hardwareMap.get(Servo.class, "hook");
        marker = hardwareMap.get(Servo.class, "marker");
    
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        while(opModeIsActive()) {
            
            float LBSpeed =  gamepad1.left_stick_y; 
            float LFSpeed =  gamepad1.left_stick_y;
            float RBSpeed = -gamepad1.right_stick_y; 
            float RFSpeed = -gamepad1.right_stick_y;
            
            LF.setPower(LFSpeed);
            LB.setPower(LBSpeed);
            RF.setPower(RFSpeed);
            RB.setPower(RBSpeed);
            
            if (gamepad1.right_trigger > .2) {
                armDown.setPower(-.5); //retract
            } else if (gamepad1.left_trigger > .2) {
                armDown.setPower(1);
            } else if(gamepad1.b) {
                // stops arm motor
                armDown.setPower(0);
            } else if (gamepad1.y) {
                hook.setPosition(0);
                marker.setPosition(0);
            } else if (gamepad1.a) {
                hook.setPosition(.5);
            } else if (gamepad1.x) {
                marker.setPosition(.5);
            }
            
            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    
    }
}
    

