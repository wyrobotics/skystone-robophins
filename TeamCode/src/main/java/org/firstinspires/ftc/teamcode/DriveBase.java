package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.MoreMath.cot;
import static org.firstinspires.ftc.teamcode.MoreMath.squareProject;
import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = mainTelemetry;

        maxSpeed = speed;

    }

    public DriveBase(HardwareMap hardwareMap, Telemetry mainTelemetry) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = mainTelemetry;

        maxSpeed = 1;

    }

    public void teleOpMove(double x, double y, double rotate) {

        double[] velocityVec = squareProject(Math.atan2(y,x) - (Math.PI / 4));

        double r = Math.hypot(x,y);
        double rotateValue = 1 + Math.abs(rotate);

        frontLeft.setPower(maxSpeed * ((r * velocityVec[0]) + rotate) / rotateValue);
        frontRight.setPower(maxSpeed * ((r * velocityVec[1]) - rotate) / rotateValue);
        backLeft.setPower(maxSpeed * ((r * velocityVec[1]) + rotate) / rotateValue);
        backRight.setPower(maxSpeed * ((r * velocityVec[0]) - rotate) / rotateValue);

        telemetry.addData("rotate: ", rotate);
        telemetry.addData("Rotate value var: ", rotateValue);
        telemetry.addData("front left: ", frontLeft.getPower());
        telemetry.update();
    }

}
