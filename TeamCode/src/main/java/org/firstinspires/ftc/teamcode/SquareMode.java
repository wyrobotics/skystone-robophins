package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;

@Config
@TeleOp
public class SquareMode extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public double cot(double theta) {
        if ((theta == Math.PI / 2) || (theta == -Math.PI / 2)) {
            return 0;
        } else {
            return 1 / Math.tan(theta);
        }
    }

    double po4 = Math.PI / 4;

    private double[] squareProject(double theta) {
        double[] output = new double[2];
        if((theta < po4) && (theta >= -po4)) {
            output[0] = 1;
            output[1] = Math.tan(theta);
        } else if((theta < 3 * po4) && (theta >= po4)) {
            output[0] = cot(theta);
            output[1] = 1;
        } else if((theta >= 3 * po4) || (theta <= -3 * po4)) {
            output[0] = -1;
            output[1] = -Math.tan(theta);
        } else if((theta < -po4) && (theta >= -3 * po4)) {
            output[0] = -cot(theta);
            output[1] = -1;
        }
        return output;
    }




    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

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

        while(opModeIsActive()) {
            double[] leftStick = {-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y};

            //are theta gives a list of len 2 with r, theta using pythag theorem and arctan
            double[] areTheta = new double[2];
            areTheta[0] = Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]));
            areTheta[1] = Math.atan2(leftStick[1], leftStick[0]);

            double[] longSquare = squareProject(areTheta[1] - (Math.PI / 4));

            longSquare[0] = longSquare[0] * Math.pow(areTheta[0], 2);
            longSquare[1] = longSquare[1] * Math.pow(areTheta[0], 2);

            frontLeft.setPower(longSquare[0]);
            frontRight.setPower(longSquare[1]);
            backLeft.setPower(longSquare[1]);
            backRight.setPower(longSquare[0]);


            dashboardTelemetry.addData("Left Wheel: ", longSquare[0]);
            dashboardTelemetry.update();


            telemetry.update();
        }

    }

}