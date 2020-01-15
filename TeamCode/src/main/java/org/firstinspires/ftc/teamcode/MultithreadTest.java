package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MultithreadTest extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Servo rightPlatform;

    @Override
    public void runOpMode() {

        //OdometryTracker odometryTracker = new OdometryTracker(hardwareMap, telemetry);

        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        waitForStart();

        //odometryTracker.multithreadTest();

        double dir = -1;

        while(opModeIsActive()) {

            rightPlatform.setPosition(0.35 + dir * 0.2);

            sleep(1000);

            dir *= -1;

            dashboardTelemetry.addData("Bruh?? ", 2000 - System.currentTimeMillis());
            dashboardTelemetry.update();

        }

        //odometryTracker.shutdownOdometry();

    }

}
