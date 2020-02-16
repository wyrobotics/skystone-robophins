package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.Components.ExtraMath.squareProject;

public class MainRobot {

    public DriveBase driveBase;

    public OdometryTracker odometryTracker;

    public DcMotor frontLifter;
    public DcMotor backLifter;
    private CRServo extender;
    private double extenderDirection;
    private double lifterSpeed = 0.7;

    private CRServo rotator;
    private CRServo grabber;

    private Servo leftPlatform;
    private Servo rightPlatform;

    //private Servo backLeftPlatform;
    //private Servo backRightPlatform;

    private DigitalChannel extenderSwitch;
    private boolean extended;
    private boolean extenderSwitchOverride = false;
    public DigitalChannel lifterSwitch;

    private DcMotor shooter;

    private DcMotor backRotator;
    private Servo backGrabber;

    private Servo backLeftPlatformGrabber;
    private Servo backRightPlatformGrabber;

    public boolean backGrabberDown = false;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry, double speed) {

        driveBase = new DriveBase(hardwareMap, telemetry, speed);

        odometryTracker = new OdometryTracker(hardwareMap, telemetry);

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();

        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;

        frontLifter = hardwareMap.get(DcMotor.class, "frontLifter");
        backLifter = hardwareMap.get(DcMotor.class, "backLifter");
        extender = hardwareMap.get(CRServo.class, "extender");

        rotator = hardwareMap.get(CRServo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        lifterSwitch = hardwareMap.get(DigitalChannel.class, "lifterSwitch");

        lifterSwitch.setMode(DigitalChannel.Mode.INPUT);

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        backRotator = hardwareMap.get(DcMotor.class, "backRotator");
        backGrabber = hardwareMap.get(Servo.class, "backGrabber");

        backLeftPlatformGrabber = hardwareMap.get(Servo.class, "backLeftPlatformGrabber");
        backRightPlatformGrabber = hardwareMap.get(Servo.class, "backRightPlatformGrabber");

        backLeftPlatformGrabber.setDirection(Servo.Direction.REVERSE);

        backRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backGrabber.setDirection(Servo.Direction.REVERSE);

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        //backLeftPlatform.setDirection(Servo.Direction.REVERSE);
        //backRightPlatform.setDirection(Servo.Direction.FORWARD);

        frontLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //backRotate(false);

        backPlatformGrab(false);

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

    /*
    public void backRotate(boolean down) {
        backRotator.setPosition(down ? 0.1 : 1);
        backGrabberDown = down;
    }

     */

    public void backRotate(double power) {
        backRotator.setPower(0.5 * power);
    }
    public void backRotate(boolean down) {
        if(down) {
            backRotator.setPower(-0.5);
            backGrabberDown = true;
        } else {
            backRotator.setPower(0.7);
            backGrabberDown = false;
        }
    }

    public void backGrab(boolean out) { backGrabber.setPosition(out ? 0.3 : 1); }

    public void backPlatformGrab(boolean down) {
        if(down) {
            backLeftPlatformGrabber.setPosition(0.55);
            backRightPlatformGrabber.setPosition(0.55);
        } else {
            backLeftPlatformGrabber.setPosition(0.0);
            backRightPlatformGrabber.setPosition(0.0);
        }
    }

    public void overrideLimitSwitch() { extenderSwitchOverride = true; }

    public void yeet(boolean party) {
        CameraDevice.getInstance().setFlashTorchMode(party);
    }

}
