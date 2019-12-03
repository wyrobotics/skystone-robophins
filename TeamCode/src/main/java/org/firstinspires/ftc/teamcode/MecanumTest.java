package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class MecanumTest extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    //during init
    public void runOpMode(){
        frontLeft= hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");
        backLeft= hardwareMap.get(DcMotor.class, "backLeft");
        backRight= hardwareMap.get(DcMotor.class, "backRight");

        telemetry.addData("Status:", "Running");
        telemetry.update();

        //set zero power behavior to brake stops the motors from moving at the beginning
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //stop and reset encoder sets the values back to zero upon restarting the robot (or is it initialization? i forget)
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        //during play button pressed
        while(opModeIsActive()){


            move(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x, this.gamepad1.right_stick_y);

            //print statements
            //shows power values
            telemetry.addData("-----POWER------", "");
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Front Right:", frontRight.getPower());
            telemetry.addData("Back Left:", backLeft.getPower());
            telemetry.addData("Back Right:", backRight.getPower());

            //shows encoder position values (this is how you find your values for auton)
            //getCurrentPosition() specifically does that as per the name :s
            telemetry.addData("-----ENCODERS------", "");
            telemetry.addData("Front Left:", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", frontRight.getCurrentPosition());
            telemetry.addData("Back Left:", backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", backRight.getCurrentPosition());

            /*telemetry.addData("-----PORT------", "");
            telemetry.addData("Front Left:", frontLeft.getPortNumber());
            telemetry.addData("Front Right:", frontRight.getPortNumber());
            telemetry.addData("Back Left:", backLeft.getPortNumber());
            telemetry.addData("Back Right:", backRight.getPortNumber());*/

            //update print
            telemetry.update();
        }
    }
    /*forward: all forward
    backward: all backward
    strafe:  left: fr bl forward
                    fl br backward
             right: fl br forward
                    fr bl backward
    turn: like tank drive; right: left side forward, right side backward
                           left: right side forward, left side backward
    */
    /**fix slow turning and add diagonal motion; add speed toggle, fix reversed turning */
    public void move(double leftx, double lefty, double rightx, double righty){
        //straight line motion forwards nad backwards
        if(Math.abs(lefty) > Math.abs(leftx)){
            frontLeft.setPower(lefty);
            backLeft.setPower(lefty);
            frontRight.setPower(-lefty);
            backRight.setPower(-lefty);
        }

        else if(Math.abs(leftx) > Math.abs(lefty)) { // strafing

            backLeft.setPower(leftx); //default negative
            frontRight.setPower(-leftx);

            backRight.setPower(leftx);
            frontLeft.setPower(-leftx);

        } else if(Math.abs(rightx) > Math.abs(righty)){
            frontLeft.setPower(rightx);
            backLeft.setPower(rightx);
            frontRight.setPower(rightx); //negative
            backRight.setPower(rightx); //negative
        }
        if (Math.abs(lefty) <.2  && Math.abs(leftx) <.2){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }
}
