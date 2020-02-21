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
        sleep(500);

        moveRelative(0,10,1000);

        autonomousRobot.backRotate(-0.8);
        turn(86, true);
        autonomousRobot.backRotate(0);

        moveRelative(0,72 + 8 * strafeCounter - (strafeCounter == 0 ? 3 : 0), 3000);
        //moveRelative(0,Math.abs(-78 - autonomousRobot.odometryTracker.getPosition()[1]),3000);
        autonomousRobot.backRotate(false);
        turn(90,false);
        autonomousRobot.backRotate(0);

        moveRelative(0,-17,2000);
        autonomousRobot.backRotate(true);
        autonomousRobot.backGrab(true);
        sleep(300);
        autonomousRobot.backRotate(false);
        sleep(300);
        autonomousRobot.backGrab(false);
        autonomousRobot.backPlatformGrab(true);
        sleep(500);

        moveRelative(0,30,2700);
        turnTimeout(90,false,1500);

        moveRelative(0,-30,500);

        autonomousRobot.backPlatformGrab(false);
        sleep(300);
        moveRelative(0,70,300);
        turn(190,true);
        autonomousRobot.shoot(1);
        sleep(1000);
        autonomousRobot.shoot(0);

    }

}
