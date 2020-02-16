package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Components.AutonomousRobot;

@Autonomous
public class AutonRed extends AutonomousOpMode {

    public void runOpMode() {

        autonomousRobot = new AutonomousRobot(hardwareMap, telemetry, 1);

        flashlight(true);

        waitForStart();

        autonomousRobot.odometryTracker.startOdometry();

        moveRelative(0,-20,3000);

        autonomousRobot.skyStones.activate();

        skystoneSleep(300);

        double strafeCounter = 0;

        if(!autonomousRobot.skyStoneFinder.isVisible()) {
            strafeCounter += 1;
            strafeProfiled(8,true);
            skystoneSleep(300);
            if(!autonomousRobot.skyStoneFinder.isVisible()) {
                strafeCounter += 1;
            }
        }
        if(strafeCounter != 2) { strafeProfiled(8,false); }

        flashlight(false);

        autonomousRobot.backRotate(-0.7);
        autonomousRobot.backGrab(true);
        sleep(1500);
        autonomousRobot.backRotate(0);

        moveRelative(0,-10,2000);

        autonomousRobot.backGrab(false);
        sleep(500);
        autonomousRobot.backRotate(false);
        sleep(1000);

        moveRelative(0,10,1000);
        autonomousRobot.backRotate(true);
        turn(85, true);
        autonomousRobot.backRotate(0);

        moveRelative(0,69 + 8 * strafeCounter - (strafeCounter == 0 ? 3 : 0), 3000);
        autonomousRobot.backRotate(false);
        turn(90,false);
        autonomousRobot.backRotate(0);

        moveRelative(0,-19,2000);
        autonomousRobot.backRotate(true);
        autonomousRobot.backGrab(true);
        sleep(300);
        autonomousRobot.backRotate(false);
        sleep(300);
        autonomousRobot.backGrab(false);
        autonomousRobot.backPlatformGrab(true);
        sleep(500);

        moveRelative(0,30,3000);
        turnTimeout(90,false,2000);

        moveRelative(0,-30,1000);

        autonomousRobot.backPlatformGrab(false);
        sleep(300);
        moveRelative(0,12,1000);
        turn(180,true);
        autonomousRobot.shoot(1);
        sleep(1000);
        autonomousRobot.shoot(0);

    }

}
