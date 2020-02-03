package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Components.ExtraMath.squareProject;

public class MainRobot {

    public DriveBase driveBase;

    public OdometryTracker odometryTracker;

    private BNO055IMU imu;

    public DcMotor frontLifter;
    public DcMotor backLifter;
    private CRServo extender;
    private double extenderDirection;
    private double lifterSpeed = 0.4;

    private CRServo rotator;
    private CRServo grabber;

    private Servo leftPlatform;
    private Servo rightPlatform;

    //private Servo backLeftPlatform;
    //private Servo backRightPlatform;

    private DigitalChannel extenderSwitch;
    private boolean extended;
    private boolean extenderSwitchOverride = false;
    public TouchSensor lifterSwitch;

    private DcMotor shooter;

    private Servo backRotator;
    private Servo backGrabber;

    private Servo backPlatformGrabber;

    private boolean backGrabberDown;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry, double speed) {

        driveBase = new DriveBase(hardwareMap, telemetry, speed);

        odometryTracker = new OdometryTracker(hardwareMap, telemetry);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();

        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parametersIMU);

        frontLifter = hardwareMap.get(DcMotor.class, "frontLifter");
        backLifter = hardwareMap.get(DcMotor.class, "backLifter");
        extender = hardwareMap.get(CRServo.class, "extender");

        rotator = hardwareMap.get(CRServo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        lifterSwitch = hardwareMap.get(TouchSensor.class, "lifterSwitch");

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        backRotator = hardwareMap.get(Servo.class, "backRotator");
        backGrabber = hardwareMap.get(Servo.class, "backGrabber");

        backPlatformGrabber = hardwareMap.get(Servo.class, "backPlatformGrabber");

        backGrabber.setDirection(Servo.Direction.REVERSE);

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        //backLeftPlatform.setDirection(Servo.Direction.REVERSE);
        //backRightPlatform.setDirection(Servo.Direction.FORWARD);

        frontLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        backRotate(false);

        backPlatformRelease();

    }

    public void lift(double power) {
        if(!lifterSwitch.isPressed()) {
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

        if(extenderSwitchOverride) {
            extender.setPower(power);
        } else {
            if (!extenderSwitch.getState() && !extended) {
                extenderDirection = extender.getPower();
                extended = true;
                newPower = 0;
            } else {
                newPower = power;
            }

            if (extended) {
                newPower = (extenderDirection > 0) ? Math.min(0, power) : Math.max(0, power);
            }

            if (extended && extenderSwitch.getState()) {
                extended = false;
            }

            extender.setPower(newPower);
        }

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

    /*
    public void backGrabPlatform() {
        backLeftPlatform.setPosition(0.4);
        backRightPlatform.setPosition(0.4);
    }

    public void backReleasePlatform() {
        backLeftPlatform.setPosition(0.5);
        backRightPlatform.setPosition(0.5);
    }

     */

    public void backRotate(boolean down) {
        backRotator.setPosition(down ? 0.1 : 1);
        backGrabberDown = down;
    }

    public void backGrab(boolean out) { backGrabber.setPosition(out ? 0.3 : 1); }

    public void backPlatformGrab() { backPlatformGrabber.setPosition(0.55); }

    public void backPlatformRelease() { backPlatformGrabber.setPosition(0.0); }

    public void overrideLimitSwitch() { extenderSwitchOverride = true; }

}
