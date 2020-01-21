package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OdomTest extends LinearOpMode {

    OdometryTracker odometryTracker;

    @Override
    public void runOpMode(){

        odometryTracker = new OdometryTracker(hardwareMap, telemetry);



        waitForStart();

        odometryTracker.startOdometry();

        while (opModeIsActive()) {

            double[] pos = odometryTracker.getPosition();

            telemetry.addData("X position", pos[0]);
            telemetry.addData("Y position", pos[1]);
            telemetry.addData("Heading (degrees)", pos[2] * 360 / (2 * Math.PI));
            //telemetry.addData("Left Pos: ", odometryTracker.leftEncoder.getCurrentPosition());
            //telemetry.addData("Right Pos: ", odometryTracker.rightEncoder.getCurrentPosition());
            telemetry.update();

        }

        odometryTracker.shutdownOdometry();

    }

}
