package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Components.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Components.AutonomousRobot;

@Autonomous
public class AutonRedOdom extends AutonomousOpMode {

    public void runOpMode() {

        autonomousRobot = new AutonomousRobot(hardwareMap, telemetry, 0.7);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aafi8Rf/////AAABmZaaeBrQs0yjh3eFx75cZcl73N2MA5Via6esnTbOYG2Nrdhk5SICeeNhRlq6GxGlcYzMJrz9cRielxWdCT+KH+pQ3gJNk+kCxdj8PW32skELsyj16/pt72QG4AiYlp8U4rdAYcuaxyMwe2ST3Dw8ZphuioeZJYidhtcioGiO/fjc/EQvLDZ4qHK0LtryQQi3Kt3mlqYqEVQXsswOHiRAR2NLtoLut5l/sdRYfL/pumIckZRmE56XNcjBZUQNeP+IMs0dkCJ/8hoPmGyEcw6Ijnc88sMvrih3sjuUEPx99nC94UBAzJPdZeeFLa6jNxLQdRI23enRnGM04NuMF+F2JOeLNSKT4xxOVFw4WunORKkk";

        //i stuck with the back camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        autonomousRobot.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /**
         * Load the data containing the VuMarks for SKYSTONE (edit: there are MULTIPLE trackable objects for this game, this is ONLY THE SKYSTONE)
         * @see VuMarkInstanceId
         */

        VuforiaTrackables skyStones = this.autonomousRobot.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable skyStone = skyStones.get(0);

        VuforiaTrackableDefaultListener skyStoneFinder = new VuforiaTrackableDefaultListener();

        skyStoneFinder.addTrackable(skyStone);


        skyStone.setListener(skyStoneFinder);



        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        FtcDashboard.getInstance().startCameraStream(autonomousRobot.vuforia,30);




        CameraDevice.getInstance().setFlashTorchMode(true);



        waitForStart();

        autonomousRobot.odometryTracker.startOdometry();

        /**
         *  RED
         *  movement sequence
         *      start facing backwards
         *      go BACKWARDS 1
         *      right .2 to detect
         *      if not found, strafe .3tiles left w/correction
         *
         *      after found:
         *          then bring back grabber down and open
         *          then go forward and grab
         *          then pick up block
         *      turn left 90
         *      go forward 3.1-3.2 tiles plus 0-.6 depending on which stone is the skystone
         *      turn right 90, go backward .5
         *
         *      bring grabber down
         *      let go of block
         *      back up .3
         *      rotate 180
         *      go forward
         *      grab plate
         *      turn right until perpendicular to wall
         *      go forward .5
         *      bring pullers up
         *      back up .3
         *      strafe left .3
         *      tape shooter (while strafing if low on time)
         *
         *
         */

        //moveStrafe(6, false);
        moveRelative(12,-20, 5000);

        skyStones.activate();

        sleep(400);



        int counter = 0;
        while(!skyStoneFinder.isVisible() && counter < 2){
            counter += 1;
            moveStrafe(7.1, true);
            sleep(400);
        }

        moveStrafe(6, true);

        turnTo(0,autonomousRobot.odometryTracker.getAngle() < 0, 2000);

        autonomousRobot.backRotate(true);
        autonomousRobot.backGrab(true);
        sleep(200);
        moveRelative(0, -13,2000);
        autonomousRobot.backGrab(false);
        sleep(500);
        autonomousRobot.backRotate(false);
        moveRelative(0,8,2000);

        turnTo(-85, false, 2000); //ccw (left) positive delta theta
        autonomousRobot.backRotate(true);
        sleep(100);
        moveRelative(0, -(84 - counter*8),2500);
        autonomousRobot.backRotate(false);
        turn(90,true);
        moveRelative(0,-18 - ((counter == 2) ? 6 : 0),2500);

        autonomousRobot.backRotate(true);
        sleep(500);
        autonomousRobot.backGrab(true);
        sleep(500);

        autonomousRobot.backGrab(false);
        autonomousRobot.backRotate(false);

        sleep(300);

        autonomousRobot.backPlatformGrab();

        sleep(800);

        moveRelative(0,30,2000);

        turnTimeout(90,false,3000);

        /*
        moveRelative(0,6,2000);
        turnOdom(180, true);
        sleep(100);
        moveRelativeOdomAngle(0, 8,2000);


        autonomousRobot.grabPlatform();


        sleep(1000);
        moveRelativeOdomAngle(0,-24,2000);
        turnOdom(90,false);
        moveRelative(0,8,2000);
        autonomousRobot.releasePlatform();
        sleep(500);
        moveRelative(0, -5,1000);

*/
        autonomousRobot.shoot(1);
        sleep(2000);
        autonomousRobot.shoot(0);











        autonomousRobot.odometryTracker.shutdownOdometry();

    }

}
