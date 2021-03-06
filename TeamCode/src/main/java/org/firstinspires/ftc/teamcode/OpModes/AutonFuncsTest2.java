package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Components.AutonomousRobot;

@Autonomous
public class AutonFuncsTest2 extends AutonomousOpMode {

    public void runOpMode() {

        autonomousRobot = new AutonomousRobot(hardwareMap, telemetry, 1);

        autonomousRobot.odometryTracker.startOdometry();

        waitForStart();

        //moveRelative(24,0, 1000000);

        //moveStrafe(6, true);

        //turn(90, false);
        ////gyroturn(90, true);
        strafeProfiled(24, false);

        autonomousRobot.odometryTracker.shutdownOdometry();

        sleep(100000);

    }



}
