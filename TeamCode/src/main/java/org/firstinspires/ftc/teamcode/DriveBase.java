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

    public DriveBase(HardwareMap hardwareMap, Telemetry telemetry) {

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

    }

    public void setVelocity(double x, double y) {

        double[] velocityVec = squareProject(Math.atan2(y,x));

        frontLeft.setPower(velocityVec[0]);
        frontRight.setPower(velocityVec[1]);
        backLeft.setPower(velocityVec[1]);
        backRight.setPower(velocityVec[0]);

    }

}
