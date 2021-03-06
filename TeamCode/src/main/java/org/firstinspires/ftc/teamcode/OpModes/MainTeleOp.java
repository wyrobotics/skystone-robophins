package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.MainRobot;

import static org.firstinspires.ftc.teamcode.Components.ExtraMath.squareProject;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    private MainRobot mainRobot;

    @Override
    public void runOpMode() {

        mainRobot = new MainRobot(hardwareMap, telemetry, 1.0);

        boolean backGrabberUp = true;

        waitForStart();

        mainRobot.odometryTracker.startOdometry();

        while(opModeIsActive()) {

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

            if(this.gamepad1.x && !this.gamepad1.y) {
                mainRobot.grabPlatform();
            } else if(this.gamepad1.y) {
                mainRobot.releasePlatform();
            }

            if(this.gamepad2.x) {
                mainRobot.rotate(0.5);
            }

            mainRobot.extend(-this.gamepad2.left_stick_y);

            mainRobot.lift(this.gamepad2.right_stick_y);

            mainRobot.rotate(this.gamepad2.left_stick_x);

            if((this.gamepad2.left_trigger > 0) || (this.gamepad2.right_trigger > 0)) { mainRobot.grab(this.gamepad2.left_trigger > 0); } else { mainRobot.grab(0);}

            mainRobot.shoot(this.gamepad1.dpad_up ? 1 : (this.gamepad1.dpad_down ? -1 : 0));

            if(this.gamepad2.dpad_down || this.gamepad2.dpad_up) { mainRobot.backRotate(this.gamepad2.dpad_down ? -1 : 1); }
            else { mainRobot.backRotate(0); }

            if(this.gamepad2.dpad_left || this.gamepad2.dpad_right) { mainRobot.backGrab(this.gamepad2.dpad_right); }

            if(this.gamepad2.back) { mainRobot.overrideLimitSwitch(); }

            if(this.gamepad1.dpad_left || this.gamepad1.dpad_right) { mainRobot.backPlatformGrab(this.gamepad1.dpad_left); }




            telemetry.addData("X-pos: ", mainRobot.odometryTracker.getPosition()[0]);
            telemetry.addData("Y-pos: ", mainRobot.odometryTracker.getPosition()[1]);
            telemetry.addData("Heading ", mainRobot.odometryTracker.getPosition()[2] * 360 / (2 * Math.PI));
            telemetry.addData("Touch sensor: ", mainRobot.lifterSwitch.getState());
            telemetry.addData("Back grabber down: ", mainRobot.backGrabberDown);



            //power
            telemetry.addData("-------------------", "");
            telemetry.addData("","");
            telemetry.addData("front left: ", mainRobot.driveBase.frontLeft.getPower());
            telemetry.addData("back left: ", mainRobot.driveBase.backLeft.getPower());
            telemetry.addData("front right: ", mainRobot.driveBase.frontRight.getPower());
            telemetry.addData("back right: ", mainRobot.driveBase.backRight.getPower());
            telemetry.update();

        }

        mainRobot.odometryTracker.shutdownOdometry();

    }

}

