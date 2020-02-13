package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.Components.ExtraMath.squareProject;

public class DriveBase {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;;
    public DcMotor backRight;

    Telemetry telemetry;

    public double maxSpeed;

    public DriveBase(HardwareMap hardwareMap, Telemetry mainTelemetry, double speed) {

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

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = mainTelemetry;

        maxSpeed = speed;

    }

    public void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft.setPower(frontLeft * maxSpeed);
        this.frontRight.setPower(frontRight * maxSpeed);
        this.backLeft.setPower(backLeft * maxSpeed);
        this.backRight.setPower(backRight * maxSpeed);
    }

    public void setMotorPowers(double[] powers) {
        this.frontLeft.setPower(powers[0] * maxSpeed);
        this.frontRight.setPower(powers[1] * maxSpeed);
        this.backLeft.setPower(powers[2] * maxSpeed);
        this.backRight.setPower(powers[3] * maxSpeed);
    }

    public double frontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }
    public double frontRightPosition() {
        return frontRight.getCurrentPosition();
    }
    public double backLeftPosition() {
        return backLeft.getCurrentPosition();
    }
    public double backRightPosition() {
        return backRight.getCurrentPosition();
    }

    public double[] stickToMotorPowers(double x, double y) {
        double[] leftStick = {x, y};

        //are theta gives a list of len 2 with r, theta using pythag theorem and arctan
        double[] areTheta = new double[2];
        areTheta[0] = Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]));
        areTheta[1] = Math.atan2(leftStick[1], leftStick[0]);

        double[] longSquare = squareProject(areTheta[1] - (Math.PI / 4));

        longSquare[0] = longSquare[0] * Math.pow(areTheta[0],2);
        longSquare[1] = longSquare[1] * Math.pow(areTheta[0],2);

        double frontLeft = longSquare[0];
        double frontRight = longSquare[1];
        double backLeft = longSquare[1];
        double backRight = longSquare[0];

        return new double[] {frontLeft, frontRight, backLeft, backRight};
    }

    public void controlDriveBase (double x, double y) {
        double[] motorPowers = stickToMotorPowers(x,y);
        setMotorPowers(motorPowers[0] * maxSpeed,motorPowers[1] * maxSpeed,
                motorPowers[2] * maxSpeed,motorPowers[3] * maxSpeed);
    }

}