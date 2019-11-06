//these are functions from 2018's robot autonomous

public class autonomousFunction{

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
    
        //enables the bot to drive, putting in speed, left and right inches to move, and a timeout length
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
    //allows the robot to turn a fixed ammount.  As of 11-5-19 we have a much more useful and efficent turning function
        public void turn(){
    encoderDrive(.3, -4, -4.5, 5);
    sleep(1000);
    encoderDrive(.3, -6, 6, 5);
    sleep(1000);  
    encoderDrive(.3, 13, 13, 5);
    sleep(1000);
    encoderDrive(.3, -6, 6, 5);
    sleep(1000);
    encoderDrive(.3, 4, 4, 5); 
    }
    
}
