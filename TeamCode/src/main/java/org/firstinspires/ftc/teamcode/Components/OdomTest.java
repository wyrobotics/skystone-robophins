package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.OdometryTracker;

@TeleOp
@Disabled
public class OdomTest extends LinearOpMode {

    private OdometryTracker odometryTracker;

    @Override
    public void runOpMode(){

        odometryTracker = new OdometryTracker(hardwareMap, telemetry);



        waitForStart();

        odometryTracker.startOdometry();

        while (opModeIsActive()) {

            double[] pos = odometryTracker.getPosition();

            telemetry.addData("X position", pos[0]);
            telemetry.addData("Y position", pos[1]);
            telemetry.addData("Heading (degrees)", Math.floor(pos[2] * 360 / (2 * Math.PI)));
            telemetry.addData("Left Pos: ", odometryTracker.leftEncoder.getCurrentPosition());
            telemetry.addData("Right Pos: ", odometryTracker.rightEncoder.getCurrentPosition());
            telemetry.update();

        }

        odometryTracker.shutdownOdometry();

    }

}
