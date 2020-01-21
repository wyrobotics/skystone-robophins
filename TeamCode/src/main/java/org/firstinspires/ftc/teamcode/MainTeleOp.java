package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;

public class MainTeleOp extends LinearOpMode {

    private MainRobot mainRobot;

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

        mainRobot = new MainRobot(hardwareMap, telemetry, 0.7);

        waitForStart();

        while(opModeIsActive()) {

            double rotatorInc = 0.0;

            double[] leftStick = {this.gamepad1.left_stick_x, -this.gamepad1.left_stick_y};

            //are theta gives a list of len 2 with r, theta using pythag theorem and arctan
            double[] areTheta = new double[2];
            areTheta[0] = Math.sqrt((leftStick[0] * leftStick[0]) + (leftStick[1] * leftStick[1]));
            areTheta[1] = Math.atan2(leftStick[1], leftStick[0]);

            double[] longSquare = squareProject(areTheta[1] - (Math.PI / 4));

            longSquare[0] = longSquare[0] * Math.pow(areTheta[0],2);
            longSquare[1] = longSquare[1] * Math.pow(areTheta[0],2);

            double rotationScaler = 1 + Math.abs(this.gamepad1.right_stick_x);

            double frontLeft = (0.7 * (longSquare[0] + this.gamepad1.right_stick_x)/rotationScaler);
            double frontRight = (0.7 * (longSquare[1] - this.gamepad1.right_stick_x)/rotationScaler);
            double backLeft = (0.7 * (longSquare[1] + this.gamepad1.right_stick_x)/rotationScaler);
            double backRight = (0.7 * (longSquare[0] - this.gamepad1.right_stick_x)/rotationScaler);

            mainRobot.driveBase.setMotorPowers(frontLeft, frontRight, backLeft, backRight);

            if(this.gamepad1.a && !this.gamepad2.b) {
                mainRobot.grabPlatform();
            } else if(this.gamepad2.b) {
                mainRobot.releasePlatform();
            }

            if(this.gamepad2.x) {
                mainRobot.rotate(0.5);
            }

            mainRobot.extend(-this.gamepad2.left_stick_y);

            mainRobot.lift(-this.gamepad2.right_stick_y);

            if((0 != this.gamepad2.right_trigger) && !(0 != this.gamepad2.left_trigger)) {
                rotatorInc = 0.01;
            } else if((0 != this.gamepad2.left_trigger)) {
                rotatorInc = -0.01;
            } else {
                rotatorInc = 0;
            }
            mainRobot.rotate(rotatorInc, true);

            if(this.gamepad2.a || this.gamepad2.b) { mainRobot.grab(this.gamepad2.a); }

            mainRobot.shoot(this.gamepad2.dpad_up ? 1 : (this.gamepad2.dpad_down ? -1 : 0));

        }

    }

}

