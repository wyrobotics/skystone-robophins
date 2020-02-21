package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Components.AutonomousRobot;

@Autonomous
public class AutonBlue extends AutonomousOpMode {

    public void runOpMode() {

        autonomousRobot = new AutonomousRobot(hardwareMap, telemetry, 1);

        flashlight(true);

        waitForStart();

        autonomousRobot.odometryTracker.startOdometry();

        moveRelative(0,-20,3000);

        autonomousRobot.skyStones.activate();

        skystoneSleep(300);

        double strafeCounter = 2;

        if(!autonomousRobot.skyStoneFinder.isVisible()) {
            strafeCounter -= 1;
            strafeProfiledFixer(8,true);
            skystoneSleep(300);
            if(!autonomousRobot.skyStoneFinder.isVisible()) {
                strafeCounter -= 1;
            }
        }
        sleep(500);
        if(strafeCounter != 0) { strafeProfiled(8 - (strafeCounter == 2 ? 2 : 0),false); }
        //if(strafeCounter == 0 && autonomousRobot.odometryTracker.getPosition()[0] < 10)
        //{ strafeProfiled(Math.abs(10 - autonomousRobot.odometryTracker.getPosition()[0]),true); }

        flashlight(false);

        autonomousRobot.backRotate(-0.7);
        autonomousRobot.backGrab(true);


        sleep(1000);
        autonomousRobot.backRotate(0);

        moveRelative(0,-10,2000);

        autonomousRobot.backGrab(false);
        sleep(500);



        autonomousRobot.backRotate(false);
        sleep(1000);

        /**original turn/bring up sequence
        moveRelative(0,10,1000);
        autonomousRobot.backRotate(true);
        turn(88 + (strafeCounter == 2 ? 3 : 0), false);
        autonomousRobot.backRotate(0);
*/

        //revised to avoid hitting other blocks in the line- swapped grabber rotation and turning
        moveRelative(0,10,1000);
        autonomousRobot.backRotate(-0.8);
        turn(88 + (strafeCounter == 2 ? 3 : 0), false);
        autonomousRobot.backRotate(0);


        moveRelative(0,80 + 8 * strafeCounter - (strafeCounter == 0 ? 3 : 0), 3000);
        autonomousRobot.backRotate(false);
        turn(90,true);
        autonomousRobot.backRotate(0);

        moveRelative(0,-14 - (strafeCounter == 1 ? 3 : 0),2000);
        autonomousRobot.backRotate(true);
        autonomousRobot.backGrab(true);
        sleep(300);
        autonomousRobot.backRotate(false);
        sleep(300);
        autonomousRobot.backGrab(false);
        autonomousRobot.backPlatformGrab(true);
        sleep(500);

        moveRelative(0,28,2500);
        turnTimeout(90,true,1500);

        moveRelative(0,-30,500);

        autonomousRobot.backPlatformGrab(false);
        sleep(300);
        moveRelative(0,12,700);
        turn(185,true);
        autonomousRobot.shoot(1);
        sleep(1150);
        autonomousRobot.shoot(0);

    }

}
