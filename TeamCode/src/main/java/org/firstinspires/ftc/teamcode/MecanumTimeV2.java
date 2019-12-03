package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;

@TeleOp
public class MecanumTimeV2 extends LinearOpMode {
//credit to jshah

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double[] transform(double[] input) {

        double[] output = new double[2];

        output[0] = input[0] + input[1];
        output[1] = input[1] - input[0];

        return output;

    }

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            double flPower;
            double frPower;
            double blPower;
            double brPower;

            double[] leftStick = {-1 * this.gamepad1.left_stick_x,this.gamepad1.left_stick_y};

            double mag = Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]));

            double[] unitLeftStick = {(leftStick[0] / Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]))),
                    (leftStick[1] / Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1])))};
            unitLeftStick = transform(unitLeftStick);
            leftStick = transform(leftStick);
            leftStick[0] = leftStick[0] / Math.sqrt((unitLeftStick[0] * unitLeftStick[0]) + (unitLeftStick[1] * unitLeftStick[1]));
            leftStick[1] = leftStick[1] / Math.sqrt((unitLeftStick[0] * unitLeftStick[0]) + (unitLeftStick[1] * unitLeftStick[1]));

            if (mag == 0) {
                flPower = 0;
                brPower = 0;
                frPower = 0;
                blPower = 0;
            } else {
                flPower = leftStick[0];
                brPower = leftStick[0];
                frPower = leftStick[1];
                blPower = leftStick[1];
            }

            double rotationScalar = 1 + Math.abs(this.gamepad1.right_stick_x);

            flPower = (flPower - gamepad1.right_stick_x)  / rotationScalar;
            brPower = (brPower + gamepad1.right_stick_x) / rotationScalar;
            frPower = (frPower + gamepad1.right_stick_x) / rotationScalar;
            blPower = (blPower - gamepad1.right_stick_x) / rotationScalar;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);
            backLeft.setPower(blPower);

            telemetry.addData("Front Left ", flPower);
            telemetry.addData("Front Right ", frPower);
            telemetry.addData("Back Left ", blPower);
            telemetry.addData("Back Right ", brPower);
            telemetry.addData("Rotation Scalar ", rotationScalar);
            telemetry.addData("Julian is cool? ", true);

            telemetry.update();

        }

    }

}
