package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;

import java.lang.Math;

@TeleOp
public class coOpMode extends LinearOpMode {

    //Drivebase motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Arm/hand stuff
    private DcMotor lifter;
    private CRServo extender;
    private Servo rotator;
    private Servo grabber;

    //Plate movers, names based on facing robot from behind
    private Servo leftPlatform;
    private Servo rightPlatform;

    private double[] transform(double[] input) {

        double[] output = new double[2];

        output[0] = input[0] + input[1];
        output[1] = input[1] - input[0];

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
        grabber = hardwareMap.get(Servo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {

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

            //init left stick input vector
            double[] leftStick = {-1 * this.gamepad1.left_stick_x,this.gamepad1.left_stick_y};

            //magnitude of input vector
            double mag = Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]));

            //creates unit vector with same argument as input vector
            double[] unitLeftStick = {(leftStick[0] / mag), (leftStick[1] / mag)};

            //unit left stick now representd transformed unit left stick
            unitLeftStick = transform(unitLeftStick);
            //left stick now representd transformed left stick
            leftStick = transform(leftStick);

            //normalizes all wheel power values to [-1,1]
            leftStick[0] = leftStick[0] / Math.sqrt((unitLeftStick[0] * unitLeftStick[0]) + (unitLeftStick[1] * unitLeftStick[1]));
            leftStick[1] = leftStick[1] / Math.sqrt((unitLeftStick[0] * unitLeftStick[0]) + (unitLeftStick[1] * unitLeftStick[1]));

            //zero is a weird number; if mag is zero manually set power to zero
            if (mag == 0) {
                flPower = 0;
                brPower = 0;
                frPower = 0;
                blPower = 0;
            } else {
                flPower = leftStick[0];
                brPower = leftStick[0];
                frPower = leftStick[1];
                blPower = leftStick[1];
            }

            //rotate
            double rotationScalar = 1 + Math.abs(this.gamepad1.right_stick_x);

            //account for rotate
            flPower = (flPower - gamepad1.right_stick_x)  / rotationScalar;
            brPower = (brPower + gamepad1.right_stick_x) / rotationScalar;
            frPower = (frPower + gamepad1.right_stick_x) / rotationScalar;
            blPower = (blPower - gamepad1.right_stick_x) / rotationScalar;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);
            backLeft.setPower(blPower);


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



            extender.setPower(-this.gamepad2.left_stick_y);

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
            }
            rotator.setPosition(Math.max(0, Math.min(1, rotator.getPosition() + rotatorInc)));

            /*
            rotator.setPosition(Math.max(rotator.getPosition(), this.gamepad2.right_trigger));
            rotator.setPosition(Math.min(rotator.getPosition(), 1 - this.gamepad2.left_trigger));

            telemetry.addData("Right Trigger: ", this.gamepad2.right_trigger);
            telemetry.addData("Left Trigger: ", 1 - this.gamepad2.left_trigger);

             */


            if(!(!aPressed ^ this.gamepad2.a)) {
                aPressed = !aPressed;
            }
            if(!(!bPressed ^ this.gamepad2.b)) {
                bPressed = !bPressed;
            }
            if(aPressed && !bPressed) {
                grabber.setPosition(0.0);
            } else if(bPressed) {
                grabber.setPosition(1.0);
            }


            telemetry.addData("Front Left: ", flPower);
            telemetry.addData("Front Right: ", frPower);
            telemetry.addData("Back Left", blPower);
            telemetry.addData("Back Right", brPower);

            telemetry.update();

        }

    }

}
