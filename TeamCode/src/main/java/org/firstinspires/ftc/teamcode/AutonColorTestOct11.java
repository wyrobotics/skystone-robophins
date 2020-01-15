package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@Disabled
public class AutonColorTestOct11 extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private ColorSensor cdsensor;

    @Override
    public void runOpMode(){
        frontLeft= hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");
        backLeft= hardwareMap.get(DcMotor.class, "backLeft");
        backRight= hardwareMap.get(DcMotor.class, "backRight");

        //color distance sensor, 'imu' in hardwaremap
        cdsensor = hardwareMap.get(ColorSensor.class, "imu");




        telemetry.addData("Status:", "Running");
        telemetry.update();

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()){

            move(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x, this.gamepad1.right_stick_y);

            cdsensor.enableLed(true);

            telemetry.addData("-----encoders", frontLeft.getCurrentPosition());
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Front Right:", frontRight.getPower());
            telemetry.addData("Back Left:", backLeft.getPower());

            //YELLOW: RGB RED-255, GREEN 255, BLUE 0
            //BLACK: 0 0 0
            telemetry.addData("color sensor rgb: ", cdsensor.argb());
            telemetry.addData("color sensor light: ", cdsensor.alpha());

            telemetry.addData("-------", "-----");

            telemetry.addData("color RED: ", cdsensor.red());
            telemetry.addData("color GREEN: ", cdsensor.green());
            telemetry.addData("color BLUE: ", cdsensor.blue());


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

        }
        if(Math.abs(rightx) > Math.abs(righty)){
            frontLeft.setPower(rightx);
            backLeft.setPower(rightx);
            frontRight.setPower(rightx);
            backRight.setPower(rightx);
        }
        if (Math.abs(lefty) <.2  && Math.abs(leftx) <.2){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }




}
