package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.MoreMath.cot;

public class MainRobot {

    public DriveBase driveBase;

    public DcMotor lifter;
    public CRServo extender;

    private Servo rotator;
    private CRServo grabber;

    private Servo leftPlatform;
    private Servo rightPlatform;

    private Servo leftSlapper;
    private Servo rightSlapper;

    private DigitalChannel extenderSwitch;
    private DigitalChannel lifterSwitch;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry, double speed) {

        driveBase = new DriveBase(hardwareMap, telemetry, speed);

        lifter = hardwareMap.get(DcMotor.class, "lifter");
        extender = hardwareMap.get(CRServo.class, "extender");

        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        leftSlapper = hardwareMap.get(Servo.class, "leftSlapper");
        rightSlapper = hardwareMap.get(Servo.class, "rightSlapper");

        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        lifterSwitch = hardwareMap.get(DigitalChannel.class, "lifterLimitSwitch");

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(0.5);

    }

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        driveBase = new DriveBase(hardwareMap, telemetry);

        lifter = hardwareMap.get(DcMotor.class, "lifter");
        extender = hardwareMap.get(CRServo.class, "extender");

        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        leftSlapper = hardwareMap.get(Servo.class, "leftSlapper");
        rightSlapper = hardwareMap.get(Servo.class, "rightSlapper");

        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        lifterSwitch = hardwareMap.get(DigitalChannel.class, "lifterLimitSwitch");

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(0.5);

    }





    public void lift(double power) {
        if(lifterSwitch.getState()) {
            lifter.setPower(power);
        } else {
            lifter.setPower(1);
        }
    }

    public void extend(double power) {
        if(extenderSwitch.getState()) {
            extender.setPower(power);
        } else {
            extender.setPower(Math.min(0,power));
        }
    }

    public void grab(boolean in) {
        grabber.setPower(in ? -1 : 1);
    }

    public void rotate(double pos) {
        rotator.setPosition(pos);
    }

    public void rotate(double inc, boolean ccw) {
        rotator.setPosition(rotator.getPosition() + ((ccw ? 1 : -1) * inc));
    }



}
