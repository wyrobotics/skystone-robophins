package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled

public class BiggerBrainMecanum extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void runOpMode(){
        frontLeft= hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");
        backLeft= hardwareMap.get(DcMotor.class, "backLeft");
        backRight= hardwareMap.get(DcMotor.class, "backRight");

        telemetry.addData("Status:", "Running");
        telemetry.update();

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()){


            leftmove(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y);
            rightmove(this.gamepad1.right_stick_x);

            telemetry.addData("-----POWER------", "");
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Front Right:", frontRight.getPower());
            telemetry.addData("Back Left:", backLeft.getPower());
            telemetry.addData("Back Right:", backRight.getPower());

            telemetry.addData("-----ENCODERS------", "");
            telemetry.addData("Front Left:", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", frontRight.getCurrentPosition());
            telemetry.addData("Back Left:", backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", backRight.getCurrentPosition());

            telemetry.addData("-----PORT------", "");
            telemetry.addData("Front Left:", frontLeft.getPortNumber());
            telemetry.addData("Front Right:", frontRight.getPortNumber());
            telemetry.addData("Back Left:", backLeft.getPortNumber());
            telemetry.addData("Back Right:", backRight.getPortNumber());

            telemetry.update();
        }
    }
    /*forward: all forward
    backward: all backward
    strafe:  left: fr bl forward
                    fl br backwardy
             right: fl br forward
                    fr bl backward
    turn: like tank drive; right: left side forward, right side backward
                           left: right side forward, left side backward
    */
    /**fix slow turning and add diagonal motion; add speed toggle, fix reversed turning */
    //IF THE MOTORS ARE REVERSED PROPERLY
    public void leftmove(double leftx, double lefty){
        //straight line motion forwards nad backwards

        backRight.setPower(lefty+leftx);
        frontLeft.setPower(lefty+leftx);

        frontRight.setPower(lefty-leftx);
        backLeft.setPower(lefty-leftx);

    }
    public void rightmove(double rightx){
        //straight line motion forwards nad backwards
        frontLeft.setPower(rightx);
        backLeft.setPower(rightx);

        frontRight.setPower(-rightx);
        backRight.setPower(-rightx);

    }
}
