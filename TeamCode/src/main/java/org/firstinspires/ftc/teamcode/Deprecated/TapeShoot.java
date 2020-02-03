package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Components.AutonomousRobot;

@Autonomous
public class TapeShoot extends AutonomousOpMode {

    public void runOpMode() {

        autonomousRobot = new AutonomousRobot(hardwareMap, telemetry,1);

        waitForStart();

        autonomousRobot.shoot(1);
        sleep(1000);
        autonomousRobot.shoot(0);


    }

}
