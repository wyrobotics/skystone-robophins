package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Components.AutonomousRobot;

@Autonomous
public class BluePlatform extends AutonomousOpMode {

    public void runOpMode() {

        autonomousRobot = new AutonomousRobot(hardwareMap,telemetry,1);

        waitForStart();

        autonomousRobot.odometryTracker.startOdometry();

        moveRelative(-6,36,5000);

        moveRelative(0,4,1000);

        autonomousRobot.grabPlatform();

        sleep(2000);

        moveRelative(0,-24,5000);

        turnTimeout(90,true, 7000);

        moveRelative(0,24,5000);

        autonomousRobot.releasePlatform();

        sleep(200);

        moveRelative(0,-6,2000);

        //turnTo(90,false, 2000);

        autonomousRobot.shoot(1);
        sleep(1000);
        autonomousRobot.shoot(0);

        autonomousRobot.odometryTracker.shutdownOdometry();

    }

}
