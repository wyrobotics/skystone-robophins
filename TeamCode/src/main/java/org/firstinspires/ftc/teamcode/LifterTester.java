package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class LifterTester extends LinearOpMode {
    private DcMotor arm;

    @Override
    //during init
    public void runOpMode(){
        arm= hardwareMap.get(DcMotor.class, "arm");

        telemetry.addData("Status:", "Running");
        telemetry.update();

        //set zero power behavior to brake stops the motors from moving at the beginning
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //stop and reset encoder sets the values back to zero upon restarting the robot (or is it initialization? i forget)
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        //during play button pressed
        while(opModeIsActive()){

          //  lift(this.gamepad1.right_stick_y);
            dpadlift(this.gamepad1.dpad_up, this.gamepad1.dpad_down);
            //print statements
            //shows power values
            telemetry.addData("-----POWER------", "");
            telemetry.addData("Front Left:", arm.getPower());

            //shows encoder position values (this is how you find your values for auton)
            //getCurrentPosition() specifically does that as per the name :s
            telemetry.addData("-----ENCODERS------", "");
            telemetry.addData("Front Left:", arm.getCurrentPosition());

            /*telemetry.addData("-----PORT------", "");
            telemetry.addData("Front Left:", arm.getPortNumber());
            telemetry.addData("Front Right:", frontRight.getPortNumber());
            telemetry.addData("Back Left:", backLeft.getPortNumber());
            telemetry.addData("Back Right:", backRight.getPortNumber());*/

            //update print
            telemetry.update();
        }
    }


    public void lift(double righty){
        arm.setPower(.4*-righty);
    }


    public void dpadlift(boolean up, boolean down){
        if(up){
            arm.setPower(-.4);
        } else if(down){
            arm.setPower(.4);
        }
        arm.setPower(0);
    }
}
