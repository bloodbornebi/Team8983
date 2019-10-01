/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Drive Path (Scrim)", group="Pushbot")

public class AutonomousDP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

// Initialize the hardware variables. Note that the strings used here as parameters.  NOTE : String is same as phone motor names 
        LF  = hardwareMap.get(DcMotor.class, "leftfront");
        RF = hardwareMap.get(DcMotor.class, "rightfront");
        LB  = hardwareMap.get(DcMotor.class, "leftback");
        RB = hardwareMap.get(DcMotor.class, "rightback");
        
        
        // Most robots need the motor on one side to be reversed to drive forward - Reverse the motor that runs backwards when connected directly to the battery
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE)

        }
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

    //play space is 12ft x 12ft [144in x 144in]
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


   public void turn(double speed, double leftInches, double rightInches){
        
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;
       

       
       
   }
    
    public void firstPath(){
    
        encoderDrive(0.5, 18, 18, 5.0); 
        //pickup function
        endoderDrive(0.3, 12, 12, 5.0);
        turn();
    }
    
}
