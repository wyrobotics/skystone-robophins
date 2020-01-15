package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.PIDConstants.Kd;
import static org.firstinspires.ftc.teamcode.PIDConstants.Ki;
import static org.firstinspires.ftc.teamcode.PIDConstants.Kp;

@Autonomous
public class Auton_Red_test extends LinearOpMode {

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

    private DcMotor shooter;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    private double strafeTarget = 950;
    private double forwardTarget = 520; //experimental: 2178, but rolls
    private double liftTarget = 850; //NEED VALUE FOR THIS
    private double turnTarget = 2150;
    private double bridgeHeight = 270; // NEED REAL VALUE THIS IS BOTH BRDGEHEGHT AND MOVING ALL THE WAY DOWN

    private double startAngle;

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

        shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();

        parametersIMU.mode = BNO055IMU.SensorMode.IMU;
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = false;



        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parametersIMU);

        startAngle = getAngle();

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


        skyStone.setListener(skyStoneFinder);

        //i have no idea what this line does, but the sample code said it was helpful for debugging and not necessary
        //skyStoneTemplate.setName("skyStoneTemplate");

        skyStones.activate();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        FtcDashboard.getInstance().startCameraStream(vuforia,30);


        skyStones.activate();


        /**
         * may or may not fix the crash
         */

        AtomicBoolean running = new AtomicBoolean(false);


        waitForStart();

        while(opModeIsActive()){
            rightPlatform.setPosition(0.175);
            leftPlatform.setPosition(0.175);


            moveStraight(1.15, true, 0.4);


            int strafeDistance = 0;

            long detectionSleep = 350;
            sleep(500);

            /*
             *
             - block one: detect and strafe left
             - block two: right, detect, left
             - block three: right right


             check whether is visible each time before hand:
             depending on which block, run a specific sequence of events
             */

            if(skyStoneFinder.isVisible()){
                //strafe counter will always be zero here
                moveStrafe(.25, false);
            } else{
                strafeDistance += 1;
                moveStrafe(.333, true);
                sleep(detectionSleep);
                if(skyStoneFinder.isVisible()){
                    moveStrafe(.3, false);
                } else{
                    moveStrafe(.2, true);
                    strafeDistance += 1;
                    //moveStrafe(.1, true);
                }
            }
            if(strafeDistance!=1) {
                moveStraight(.08, true, .4);
            }





            /**Can change to implement turnAngle or PID or something
             turn(6 * strafeDistance - 1, true);
             moveStraight(0.1 + strafeDistance * .07 + (strafeDistance == 2 ? 0.07 : 0), true, 0.4);
             **/

            /**
             * now bring block over
             */


            lift(liftTarget, true); //move up to go over blocc


            long startTime = System.currentTimeMillis();
            while (opModeIsActive() && extenderSwitch.getState() && ((System.currentTimeMillis() - startTime) < 2500)) {
                extender.setPower(1.0);
            }

            //   telemetry.addData("start time: ", startTime);
            // telemetry.addData("runtine: ", System.currentTimeMillis());
            //telemetry.update();

            extender.setPower(0); //stop.

            rotator.setPosition(0); // should set to the right one


            grabber.setPower(-1);
            sleep(1200);
            grabber.setPower(0);
            //sleep(300);


            lift(liftTarget, false); // move down to grab

            //reset platform pullers
            rightPlatform.setPosition(0.6);
            leftPlatform.setPosition(0.6);
            grabber.setPower(1);
            sleep(3000);
            grabber.setPower(0);

            //moving under bridge sequence
            extender.setPower(-1);
            moveStraight(.2, false, 0.3); //old val .3, then .15
            extender.setPower(0);
            //maybe short lift sequence here
            rotator.setPosition(0.5);
            sleep(200);

            //Turns differently with a blocc, has to be slightly less than 90- was 85

            //now implementing turnangle, possibly no longer necessary
            //Should PID this
            turnAngle(90, false, 5);


            //move straight some percentage of a tile to compensate for the strafing at the beginning plus three tiles

            moveStraight(4.8 - (strafeDistance*.33) + (strafeDistance == 0 ? 0.2 : 0), true, 0.45);

            lift(liftTarget, true);
            turn(90, true);
            //  turnAngle(0, false, 5);
            //old     moveStraight(.8, true, .4);

            moveStraight(.7, true, .4);

            rightPlatform.setPosition(0.175);
            leftPlatform.setPosition(0.175);
            //bring lifter down later
            // lift(liftTarget - 200, false);
            extender.setPower(1);
            sleep(500);
            extender.setPower(0);
            grabber.setPower(-1);
            sleep(500);
            /**    grabber.setPower(0);

             //platform pulling sequence minus putting pullers down (done during grabber sleep sequence

             moveStraight(1.3, false, .4);    these are old
             turn(135, true);
             */
            moveStraight(1.5,false, .4);

            grabber.setPower(0);

            // turnAngle(90,true, 5);
            turn(270, false);

            moveStraight(.3, true, .4);

            rightPlatform.setPosition(0.6);
            leftPlatform.setPosition(0.6);
            //bring lifter down later

            moveStraight(0.3, false, 0.4);

            shooter.setPower(1);
            sleep(800);
            shooter.setPower(0);

            break;
        }



    }

    //2: 2, 3: 3, 5: 3, 7:

    public void moveStraight(double tiles, boolean forward, double speed){ //tileval is encoder val for one tile

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

        speed = speed * direction;

        while (opModeIsActive() &&
                (Math.abs(frontLeft.getCurrentPosition()) <= tilesWithTolerance)
                && (Math.abs(backRight.getCurrentPosition()) <= tilesWithTolerance)) {
            /*frontLeft.setPower(incrementedPower);
            backLeft.setPower(incrementedPower);
            frontRight.setPower(-incrementedPower);
            backRight.setPower(-incrementedPower);*/


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

            //speed = Math.max(-0.3, Math.min(0.3, speed + (dir * 0.007)));
            speed = Math.max(-0.3, Math.min(0.3, speed + (dir * 0140)));


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

    public void turnJank(double degrees, boolean counterclockwise, double speed){ //tileval is encoder val for one tile
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
            frontLeft.setPower(-speed);
            backLeft.setPower(-speed);
            frontRight.setPower(-speed);
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

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //tried: ZYX, XYZ, YZX, ZXY*,

        double deltaAngle = angles.secondAngle - lastAngles.secondAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;

        lastAngles = angles;

        telemetry.addData("First angle: ", angles.firstAngle);
        telemetry.update();

        return globalAngle;
    }

    public void turnAngle(double angle, boolean counterclockwise, double error) {

        turn(startAngle + angle - getAngle() - error, counterclockwise);

    }

    private double error(double targetPosition) {
        return targetPosition - getAngle();
    }

    private void turnPID(double target, boolean ccw) {

        double dir = ccw ? 1 : -1;

        double lastError = error(target);
        double lastTime = System.currentTimeMillis();
        double initTime = lastTime;

        double proportionTerm;
        double derivativeTerm;
        double integralTerm = 0;

        double u = 0;

        while(opModeIsActive() && Math.abs(error(target)) > 1 && System.currentTimeMillis() - initTime < 10000) {
            double error = error(target);
            double time = System.currentTimeMillis();
            proportionTerm = Kp * error;
            derivativeTerm = Kd * ((error - lastError) / Math.abs((lastTime - time)));
            integralTerm += Ki * error * Math.abs((lastTime - time));

            u = proportionTerm + integralTerm + derivativeTerm;

            frontLeft.setPower(-u * dir);
            frontRight.setPower(-u * dir);
            backLeft.setPower(-u * dir);
            backRight.setPower(-u * dir);

            telemetry.addData("error: ", lastError);
            telemetry.addData("front left: ", frontLeft.getPower());
            telemetry.addData("frontRight: ", frontRight.getPower());
            telemetry.addData("back left: ", backLeft.getPower());
            telemetry.addData("back right: ", backRight.getPower());

            telemetry.update();

            lastError = error;
            lastTime = time;
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

}
