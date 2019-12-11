package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;

import java.lang.Math;

@TeleOp
public class QuadrilateralMode extends LinearOpMode {

    //Drivebase motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Arm/hand stuff
    private DcMotor lifter;
    private CRServo extender;
    private Servo rotator;
    private CRServo grabber;

    //Plate movers, names based on facing robot from behind
    private Servo leftPlatform;
    private Servo rightPlatform;

    private Servo leftSlapper;
    private Servo rightSlapper;

    private DigitalChannel extenderSwitch;
    private DigitalChannel lifterSwitch;

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

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        leftSlapper.setDirection(Servo.Direction.REVERSE);
        rightSlapper.setDirection(Servo.Direction.FORWARD);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(0.5);

        waitForStart();

        while(opModeIsActive()) {

            boolean horizontalLimit = false;
            boolean verticalLimit = false;

            double flPower;
            double frPower;
            double blPower;
            double brPower;

            boolean aPressed = false;
            boolean bPressed = false;
            boolean yPressed = false;
            boolean xPressed = false;

            boolean rightTrigger = false;
            boolean leftTrigger = false;
            boolean rightTab = false;
            boolean leftTab = false;

            boolean dPadUp = false;
            boolean dPadDown = false;

            double rotatorInc = 0.0;

            double[] leftStick = {-this.gamepad1.left_stick_x, this.gamepad1.left_stick_y};

            //are theta gives a list of len 2 with r, theta using pythag theorem and arctan
            double[] areTheta = new double[2];
            areTheta[0] = Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]));
            areTheta[1] = Math.atan2(leftStick[1], leftStick[0]);

            double[] longSquare = squareProject(areTheta[1] - (Math.PI / 4));

            longSquare[0] = longSquare[0] * areTheta[0];
            longSquare[1] = longSquare[1] * areTheta[0];

            double rotationScaler = 1 + Math.abs(this.gamepad1.right_stick_x);

            frontLeft.setPower((longSquare[0] - this.gamepad1.right_stick_x)/rotationScaler);
            frontRight.setPower((longSquare[1] + this.gamepad1.right_stick_x)/rotationScaler);
            backLeft.setPower((longSquare[1] - this.gamepad1.right_stick_x)/rotationScaler);
            backRight.setPower((longSquare[0] + this.gamepad1.right_stick_x)/rotationScaler);

            if(!(!aPressed ^ this.gamepad1.a)) {
                aPressed = !aPressed;
            }
            if(!(!bPressed ^ this.gamepad1.b)) {
                bPressed = !bPressed;
            }
            if(aPressed && !bPressed) {
                rightPlatform.setPosition(0.4);
                leftPlatform.setPosition(0.4);
            } else if(bPressed) {
                rightPlatform.setPosition(0.0);
                leftPlatform.setPosition(0.0);
            }

            rightSlapper.setPosition((2 / 3) * gamepad1.right_trigger);
            leftSlapper.setPosition((2 / 3) * gamepad1.left_trigger);

            if(!(!xPressed ^ this.gamepad1.x)) {
                xPressed = !xPressed;
            }
            if(xPressed) {
                rotator.setPosition(0.5);
            }

            if(!extenderSwitch.getState()){
                horizontalLimit = true;
            } else {
                horizontalLimit = false;
            }
            if(!horizontalLimit){
                extender.setPower(-this.gamepad2.left_stick_y);
            } else{
                extender.setPower(Math.min(0, -this.gamepad2.left_stick_y));
            }

            if(!lifterSwitch.getState()) {
                verticalLimit = true;
            } else {
                verticalLimit = false;
            }
            if (!verticalLimit) {
                lifter.setPower(-0.5 * this.gamepad2.right_stick_y);
            } else {
                lifter.setPower(Math.max(0,-0.5 * this.gamepad2.right_stick_y));
            }
            telemetry.addData("Lifter limit? ", verticalLimit);

            if(!(!rightTrigger ^ (0 != this.gamepad2.right_trigger))) {
                rightTrigger = !rightTrigger;
            }
            if(!(!leftTrigger ^ (0 != this.gamepad2.left_trigger))) {
                leftTrigger = !leftTrigger;
            }
            if(rightTrigger && !leftTrigger) {
                rotatorInc = 0.002;
            } else if(leftTrigger) {
                rotatorInc = -0.002;
            } else {
                rotatorInc = 0;
            }
            rotator.setPosition(Math.max(0, Math.min(1, rotator.getPosition() + rotatorInc)));

            if(!(!aPressed ^ this.gamepad2.a)) {
                aPressed = !aPressed;
            }
            if(!(!bPressed ^ this.gamepad2.b)) {
                bPressed = !bPressed;
            }
            if(aPressed && !bPressed) {
                grabber.setPower(-1.0);
            } else if(bPressed) {
                grabber.setPower(1.0);
            } else {
                grabber.setPower(0);
            }


            telemetry.update();
        }

    }

}
