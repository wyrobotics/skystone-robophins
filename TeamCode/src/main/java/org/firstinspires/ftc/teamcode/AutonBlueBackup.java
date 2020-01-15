package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class AutonBlueBackup extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private DcMotor lifter;
    private CRServo extender;
    private Servo rotator;
    private CRServo grabber;

    //Plate movers, names based on facing robot from behind
    private Servo leftPlatform;
    private Servo rightPlatform;

    private DigitalChannel extenderSwitch;
    private DigitalChannel lifterSwitch;

    private double strafeTarget = 950;
    private double forwardTarget = 520; //experimental: 2178, but rolls
    private double liftTarget = 850; //NEED VALUE FOR THIS
    private double turnTarget = 2150;
    private double bridgeHeight = 270; // NEED REAL VALUE THIS IS BOTH BRDGEHEGHT AND MOVING ALL THE WAY DOWN

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        lifter = hardwareMap.get(DcMotor.class, "lifter");
        extender = hardwareMap.get(CRServo.class, "extender");
        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        lifterSwitch = hardwareMap.get(DigitalChannel.class, "lifterLimitSwitch");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotator.setPosition(0.5);

        rightPlatform.setPosition(0.6);
        leftPlatform.setPosition(0.6);

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Aafi8Rf/////AAABmZaaeBrQs0yjh3eFx75cZcl73N2MA5Via6esnTbOYG2Nrdhk5SICeeNhRlq6GxGlcYzMJrz9cRielxWdCT+KH+pQ3gJNk+kCxdj8PW32skELsyj16/pt72QG4AiYlp8U4rdAYcuaxyMwe2ST3Dw8ZphuioeZJYidhtcioGiO/fjc/EQvLDZ4qHK0LtryQQi3Kt3mlqYqEVQXsswOHiRAR2NLtoLut5l/sdRYfL/pumIckZRmE56XNcjBZUQNeP+IMs0dkCJ/8hoPmGyEcw6Ijnc88sMvrih3sjuUEPx99nC94UBAzJPdZeeFLa6jNxLQdRI23enRnGM04NuMF+F2JOeLNSKT4xxOVFw4WunORKkk";

        //i stuck with the back camera
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        /**
         * Load the data containing the VuMarks for SKYSTONE (edit: there are MULTIPLE trackable objects for this game, this is ONLY THE SKYSTONE)
         * @see VuMarkInstanceId
         */



        //i found the variable names in the documentation and assets folder under >FtcRobotController >src >main >assets
        // https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html (scroll all the way down)

        //i THINK this loads the file from the vuforia database?? but don't quote me onthat one
        //i recommend checking the assets folder and comparing this part of the sample code to what's here

        VuforiaTrackables skyStones = this.vuforia.loadTrackablesFromAsset("Skystone");

        //i believe this gets the first item in that list, and there should only be one item bc all skystone markers are the same
        // don't quote me on this one either
        VuforiaTrackable skyStone = skyStones.get(0);

        //i used the vuforia trackables listener interface which specifically looks for instances of certain objects
        VuforiaTrackableDefaultListener skyStoneFinder = new VuforiaTrackableDefaultListener();

        //addTrackable tells the listener to look for instances of a type of object- here it's the skystone
        skyStoneFinder.addTrackable(skyStone);

        //i have no idea what this line does, but the sample code said it was helpful for debugging and not necessary
        //skyStoneTemplate.setName("skyStoneTemplate");

        skyStones.activate();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        FtcDashboard.getInstance().startCameraStream(vuforia,30);



        waitForStart();


        rightPlatform.setPosition(0.175);
        leftPlatform.setPosition(0.175);


        moveStraight(1.05, true);





        int strafeCounter = 0;
        int strafedistance = 0;

        boolean found = false;

        while(strafeCounter < 2){ //&& !skyStoneFinder.isVisible()){
            //found = skyStoneFinder.isVisible();


            if(!skyStoneFinder.isVisible()) {
                moveStrafe(.4, true);
                //moveStrafe(.3, true);
                strafedistance +=1;
                sleep(150);
            }
            telemetry.addData("found? ",skyStoneFinder.isVisible());
            telemetry.update();

            dashboardTelemetry.addData("Found? ",skyStoneFinder.isVisible());
            dashboardTelemetry.update();

            sleep(500);
            strafeCounter += 1;
        }
        moveStrafe(.2, false); // old val: .333 with other .333

        turn(6 * strafedistance - 1,true );
        moveStraight(0.1 + strafedistance * .12,true);


        lift(liftTarget, true); //move up to go over blocc


        long startTime = System.currentTimeMillis();
        while(extenderSwitch.getState() && ((System.currentTimeMillis() - startTime) < 2500)){

            extender.setPower(1.0);
        }

        telemetry.addData("start time: ", startTime);
        telemetry.addData("runtine: ", System.currentTimeMillis());
        telemetry.update();

        extender.setPower(0); //stop.

        rotator.setPosition(1); // should set to the right one


        grabber.setPower(-1);
        sleep(1200);
        grabber.setPower(0);
        //sleep(300);


        lift(liftTarget, false); // move down to grab

        grabber.setPower(1);
        sleep(3000);
        grabber.setPower(0);
        //  lift(20, true); //lift

        telemetry.addData("visible?: ", skyStoneFinder.isVisible());

        sleep(2000);

      /*  grabber.setPower(0.5); //closed position
        sleep(1200);
        grabber.setPower(0);
        sleep(300);
        lift(10, true); //lift
        sleep(100);
        */
        //start moving over


        moveStraight(.3,false);


        while(opModeIsActive()) {
            /**
             - start on skystone side
             - move forward 1.3 or so tiles, then do lift sequence detect;
             - if not detected, strafe and continue detecting until the third stone
             - default to picking up the third stone
             - pickup sequence:
             - lift some number of counts
             - extend until limit switch rees
             - rotate 90
             - bring down:
             - bring down, grab
             - move backwards (hover)
             - turn, strafe to right a bit, enough to clear other robot on platform side but not too much to hit skybridge
             - go forward 3 tiles plus however much was strafed to the right
             - turn right, go forward; lift up a bit, lift up a bit more
             - latch and let go, sleep
             - go backwards some then turn left and go forward to push platform

             */


        }


    }

    public void moveStraight(double tiles, boolean forward){ //tileval is encoder val for one tile

        //straight line motion forwards nad backwards, uses frontLft and ackRight
        //USE FRONT RIGHT AND BACK LEFT VALUES FOR STRAFING (trials averaged for a tile strafing right)

        double tilesWithTolerance = (tiles * forwardTarget);
        int direction = 1;
        if(!forward){
            direction = -1;
        }
        double incrementedPower = direction *(forwardTarget -
                (frontLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2)
                *.0004 + .03;
        //grabber: from zero to one

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() &&
                (Math.abs(frontLeft.getCurrentPosition()) <= tilesWithTolerance)
                && (Math.abs(backRight.getCurrentPosition()) <= tilesWithTolerance)) {
            /*frontLeft.setPower(incrementedPower);
            backLeft.setPower(incrementedPower);
            frontRight.setPower(-incrementedPower);
            backRight.setPower(-incrementedPower);*/

            double speed = direction * 0.4;
            frontLeft.setPower(speed);
            backLeft.setPower(speed);
            frontRight.setPower(-speed);
            backRight.setPower(-speed);

            //shows power values
            telemetry.addData("FORWARD", "");
            telemetry.addData("-----POWER------", "");
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Front Right:", frontRight.getPower());
            telemetry.addData("Back Left:", backLeft.getPower());
            telemetry.addData("Back Right:", backRight.getPower());

            //shows encoder position values (this is how you find your values for auton)
            //getCurrentPosition() specifically does that as per the name :s
            telemetry.addData("-----ENCODERS------", "");
            telemetry.addData("Front Left:", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", frontRight.getCurrentPosition());
            telemetry.addData("Back Left:", backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", backRight.getCurrentPosition());
            telemetry.update();

        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);



    }

    // strafe defaulted to the right direction, negfative for left
    public void moveStrafe(double tiles, boolean right){ //tileval is encoder val for one tile
        int dir = 1;
        if(!right){
            dir = -1;
        }
        //straight line motion forwards nad backwards, uses frontLft and ackRight
        //USE FRONT RIGHT AND BACK LEFT VALUES FOR STRAFING (trials averaged for a tile strafing right)
        DcMotor x = frontRight;
        DcMotor y = backLeft;
        double tilesWithTolerance = (tiles * strafeTarget) - 10;

        double incrementedPower = dir * (strafeTarget -
                (x.getCurrentPosition() + y.getCurrentPosition()) / 2)
                *.00025 + .03;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed = 0;

        while (opModeIsActive() &&
                (Math.abs(x.getCurrentPosition()) <= tilesWithTolerance)
                && (Math.abs(y.getCurrentPosition()) <= tilesWithTolerance)) {
           /* frontLeft.setPower(incrementedPower);
            backLeft.setPower(-incrementedPower);
            frontRight.setPower(incrementedPower);
            backRight.setPower(-incrementedPower);*/

            speed = Math.max(-0.3, Math.min(0.3, speed + (dir * 0.007)));

            frontLeft.setPower(speed);
            backLeft.setPower(-speed);
            frontRight.setPower(speed);
            backRight.setPower(-speed);


            //shows power values
            telemetry.addData("STRAFE", "");
            telemetry.addData("-----POWER------", "");
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Front Right:", frontRight.getPower());
            telemetry.addData("Back Left:", backLeft.getPower());
            telemetry.addData("Back Right:", backRight.getPower());

            //shows encoder position values (this is how you find your values for auton)
            //getCurrentPosition() specifically does that as per the name :s
            telemetry.addData("-----ENCODERS------", "");
            telemetry.addData("Front Left:", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", frontRight.getCurrentPosition());
            telemetry.addData("Back Left:", backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", backRight.getCurrentPosition());

            telemetry.addData("---------------------------------------", "");
            telemetry.addData("lifter: ", lifter.getCurrentPosition());
            telemetry.update();

        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

    }

    public void turn(double degrees, boolean counterclockwise){ //tileval is encoder val for one tile
        int dir = 1;
        if(!counterclockwise){
            dir = -1; //ccw = all negative
        }
        //straight line motion forwards nad backwards, uses frontLft and ackRight
        //USE FRONT RIGHT AND BACK LEFT VALUES FOR STRAFING (trials averaged for a tile strafing right)
        DcMotor x = frontRight;
        DcMotor y = backLeft;
        double percentTurn = degrees / 360;
        double encoderTurnWithTolerance = (percentTurn * turnTarget) - 10;

        double incrementedPower = dir * (strafeTarget -
                (x.getCurrentPosition() + y.getCurrentPosition()) *.5) //slightly sus, fixed
                *.0003 + .03;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() &&
                (Math.abs(x.getCurrentPosition()) <= encoderTurnWithTolerance)
                && (Math.abs(y.getCurrentPosition()) <= encoderTurnWithTolerance)) {
            frontLeft.setPower(-incrementedPower);
            backLeft.setPower(-incrementedPower);
            frontRight.setPower(-incrementedPower);
            backRight.setPower(-incrementedPower);


            //shows power values
            telemetry.addData("STRAFE", "");
            telemetry.addData("-----POWER------", "");
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Front Right:", frontRight.getPower());
            telemetry.addData("Back Left:", backLeft.getPower());
            telemetry.addData("Back Right:", backRight.getPower());

            //shows encoder position values (this is how you find your values for auton)
            //getCurrentPosition() specifically does that as per the name :s
            telemetry.addData("-----ENCODERS------", "");
            telemetry.addData("Front Left:", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", frontRight.getCurrentPosition());
            telemetry.addData("Back Left:", backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", backRight.getCurrentPosition());
            telemetry.update();

        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void lift(double target, boolean up){
        int dir = 1;
        if(up){
            dir = -1;
        }
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(opModeIsActive() && Math.abs(lifter.getCurrentPosition()) < liftTarget && (lifterSwitch.getState() || up)){
            lifter.setPower(dir * 0.4);

            telemetry.addData("Lifter: ", lifter.getCurrentPosition());
            telemetry.update();

        }
        lifter.setPower(0);
    }

}
