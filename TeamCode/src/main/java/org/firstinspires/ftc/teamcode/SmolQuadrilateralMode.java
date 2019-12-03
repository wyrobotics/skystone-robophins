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

public class SmolQuadrilateralMode extends LinearOpMode {

    //Drivebase motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Arm/hand stuff
    private DcMotor lifter;
    private CRServo extender;
    private DigitalChannel extenderSwitch;
    private Servo rotator;
    private CRServo grabber;

    //Plate movers, names based on facing robot from behind
    private Servo leftPlatform;
    private Servo rightPlatform;

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
        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

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

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(0.5);

        waitForStart();

        while(opModeIsActive()) {

            boolean horizontalLimit = false;
            telemetry.addData("magnetic limit switch limit reached?" , extenderSwitch.getState());
            telemetry.addData("NOT NEGATED leftstick y, extender y: ", this.gamepad2.left_stick_y);
            telemetry.addData("horizontalLimit: ", horizontalLimit);

            telemetry.addData("rightTrigger: ", this.gamepad2.right_trigger);
            telemetry.addData("leftTrigger: ", this.gamepad2.left_trigger);
            telemetry.addData("rotator: ", rotator.getPosition());

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

            longSquare[0] = longSquare[0] * Math.pow(areTheta[0], 2);
            longSquare[1] = longSquare[1] * Math.pow(areTheta[0], 2);

            double rotationScaler = 1 + Math.abs(this.gamepad1.right_stick_x);

            frontLeft.setPower((longSquare[0] - this.gamepad1.right_stick_x)/rotationScaler);
            frontRight.setPower((longSquare[1] + this.gamepad1.right_stick_x)/rotationScaler);
            backLeft.setPower((longSquare[1] - this.gamepad1.right_stick_x)/rotationScaler);
            backRight.setPower((longSquare[0] + this.gamepad1.right_stick_x)/rotationScaler);

            if(!(!aPressed ^ this.gamepad1.a)) {
                aPressed = !aPressed;
            }
            if(!(!dPadUp ^ this.gamepad1.b)) {
                bPressed = !bPressed;
            }
            if(aPressed && !bPressed) {
                rightPlatform.setPosition(0.4);
                leftPlatform.setPosition(0.4);
            } else if(bPressed) {
                rightPlatform.setPosition(0.0);
                leftPlatform.setPosition(0.0);
            }


            //not sure whether the robot will overshoot, might have to toggle as a variable
            /*if (extenderSwitch.getState()) {
                extender.setPower(Math.max(-this.gamepad2.left_stick_y,0));
            } else {
                extender.setPower(-this.gamepad2.left_stick_y);
            }*/

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

            lifter.setPower(-0.5 * this.gamepad2.right_stick_y);

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


            telemetry.addData("rotatorInc: ", rotatorInc);
            telemetry.addData("Left Stick?", leftTrigger);
            telemetry.addData("Right Stick?", rightTrigger);


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
