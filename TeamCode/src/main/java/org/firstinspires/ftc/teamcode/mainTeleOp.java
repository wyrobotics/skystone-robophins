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
public class mainTeleOp extends LinearOpMode {

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
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

            double[] leftStick = {-1 * this.gamepad1.left_stick_x,this.gamepad1.left_stick_y};

            double mag = Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]));

            double[] unitLeftStick = {(leftStick[0] / Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]))),
                    (leftStick[1] / Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1])))};
            unitLeftStick = transform(unitLeftStick);
            leftStick = transform(leftStick);
            leftStick[0] = leftStick[0] / Math.sqrt((unitLeftStick[0] * unitLeftStick[0]) + (unitLeftStick[1] * unitLeftStick[1]));
            leftStick[1] = leftStick[1] / Math.sqrt((unitLeftStick[0] * unitLeftStick[0]) + (unitLeftStick[1] * unitLeftStick[1]));

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

            double rotationScalar = 1 + Math.abs(this.gamepad1.right_stick_x);

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
            if(!(!bPressed ^ this.gamepad1.b)) {
                bPressed = !bPressed;
            }
            if(aPressed && !bPressed) {
                grabber.setPosition(0.1);
            } else if(bPressed) {
                grabber.setPosition(1.0);
            }


            if(!(!xPressed ^ this.gamepad1.x)) {
                xPressed = !xPressed;
            }
            if(!(!yPressed ^ this.gamepad1.y)) {
                yPressed = !yPressed;
            }
            if(yPressed && !xPressed) {
                rotator.setPosition(0.5);
            } else if(xPressed) {
                rotator.setPosition(1.0);
            }



            if(!(!rightTrigger ^ (0 != this.gamepad1.right_trigger))) {
                rightTrigger = !rightTrigger;
            }
            if(!(!leftTrigger ^ (0 != this.gamepad1.left_trigger))) {
                leftTrigger = !leftTrigger;
            }
            if(rightTrigger && !leftTrigger) {
                extender.setPower(1.0);
            } else if(leftTrigger) {
                extender.setPower(-1.0);
            } else {
                extender.setPower(0.0);
            }


            if(!(!rightTab ^ this.gamepad1.right_bumper)) {
                rightTab = !rightTab;
            }
            if(!(!leftTab ^ this.gamepad1.left_bumper)) {
                leftTab = !leftTab;
            }
            if(rightTab && !leftTab) {
                lifter.setPower(0.4);
            } else if (leftTab) {
                lifter.setPower(-0.4);
            } else {
                lifter.setPower(0.0);
            }


            if(!(!dPadDown ^ this.gamepad1.dpad_down)) {
                dPadDown = !dPadDown;
            }
            if(!(!dPadUp ^ this.gamepad1.dpad_up)) {
                dPadUp = !dPadUp;
            }
            if(dPadDown && !dPadUp) {
                rightPlatform.setPosition(0.4);
                leftPlatform.setPosition(0.4);
            } else if(dPadUp) {
                rightPlatform.setPosition(0.0);
                leftPlatform.setPosition(0.0);
            }

        }

    }

}
