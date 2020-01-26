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

    public OdometryTracker odometryTracker;

    public DcMotor frontLifter;
    public DcMotor backLifter;
    private CRServo extender;
    private double extenderDirection;
    private double lifterSpeed = 0.8;

    private CRServo rotator;
    private CRServo grabber;

    private Servo leftPlatform;
    private Servo rightPlatform;

    private DigitalChannel extenderSwitch;
    private boolean extended;
    private DigitalChannel lifterSwitch;

    private DcMotor shooter;

    private Servo backRotator;
    private Servo backGrabber;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry, double speed) {

        driveBase = new DriveBase(hardwareMap, telemetry, speed);

        odometryTracker = new OdometryTracker(hardwareMap, telemetry);

        frontLifter = hardwareMap.get(DcMotor.class, "frontLifter");
        backLifter = hardwareMap.get(DcMotor.class, "backLifter");
        extender = hardwareMap.get(CRServo.class, "extender");

        rotator = hardwareMap.get(CRServo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        lifterSwitch = hardwareMap.get(DigitalChannel.class, "lifterLimitSwitch");

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        backRotator = hardwareMap.get(Servo.class, "backRotator");
        backGrabber = hardwareMap.get(Servo.class, "backGrabber");

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        frontLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRotate(false);

    }

    public void lift(double power) {
        if(lifterSwitch.getState()) {
            frontLifter.setPower(power * lifterSpeed);
            backLifter.setPower(power * lifterSpeed);
        } else {
            frontLifter.setPower(Math.min(0,power * lifterSpeed));
            backLifter.setPower(Math.min(0,power * lifterSpeed));
        }
    }


    /*
    public void extend(double power) {
        if(extenderSwitch.getState()) {
            extender.setPower(power);
        } else {
            extender.setPower(Math.max(0,power));
        }
    }*/

    public void extend(double power) {

        double newPower;

        if (!extenderSwitch.getState() && !extended) {
            extenderDirection = extender.getPower();
            extended = true;
            newPower = 0;
        } else { newPower = power; }

        if (extended) { newPower = (extenderDirection > 0) ? Math.min(0,power) : Math.max(0,power); }

        if (extended && extenderSwitch.getState()) { extended = false; }

        extender.setPower(newPower);

    }

    public void grab(boolean in) {
        grabber.setPower(in ? -1 : 1);
    }
    public void grab(double power) {grabber.setPower(power);}

    public void rotate(double power) { rotator.setPower(0.66 * power); }

    public void shoot(double power) { shooter.setPower(power); }

    public void grabPlatform() {
        leftPlatform.setPosition(0.175);
        rightPlatform.setPosition(0.175);
    }

    public void releasePlatform() {
        leftPlatform.setPosition(0.6);
        rightPlatform.setPosition(0.6);
    }

    public void backRotate(boolean down) { backRotator.setPosition(down ? 0.6 : 1);}

    public void backGrab(boolean in) { backGrabber.setPosition(in ? 1 : 0.5);}

}
