package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class DrivePark extends AutonomousOpMode {

    public void runOpMode() {

        autonomousRobot = new AutonomousRobot(hardwareMap, telemetry, 1);

        waitForStart();

        autonomousRobot.odometryTracker.startOdometry();

        sleep(1000);

        moveRelative(0,12, 5000);

        autonomousRobot.odometryTracker.shutdownOdometry();

    }

}
