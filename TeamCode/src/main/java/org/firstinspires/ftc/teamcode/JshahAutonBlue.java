/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */
@Autonomous
public class JshahAutonBlue extends LinearOpMode {
    /**note: most of this stuff is from sample code not me bc smol brain but it works so ya know
     * also note that the sample code was written with relic recovery from 2yrs ago in mind
     */

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    int DIRECTION = -1; //blue

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor lifter;
    private CRServo extender;
    private Servo rotator;
    private Servo grabber;

    private Servo rightPlatform;
    private Servo leftPlatform;


    private double strafeTarget = 2355;
    private double forwardTarget = 2178; //experimental: 2178, but rolls
    private double liftTarget = 900; //NEED VALUE FOR THIS, updated to be a bit over blocc height so it doesn't tip
    //lift target is closer to 800 now
    private double bridgeHeight = 150;//270; // NEED REAL VALUE THIS IS BOTH BRDGEHEGHT AND MOVING ALL THE WAY DOWN

    @Override public void runOpMode() {

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

        //make it go both ways
        skyStone.setListener(skyStoneFinder);

        //addTrackable tells the listener to look for instances of a type of object- here it's the skystone
        skyStoneFinder.addTrackable(skyStone);

        //i have no idea what this line does, but the sample code said it was helpful for debugging and not necessary
        //skyStoneTemplate.setName("skyStoneTemplate");


        /**
         * Movement instantiation
         *
         * disclaimer: hardware mapping is yeeted from jshah's teleop code, as is cr servo stuff
         * is entirely correct, just isn't mine
         *
         * LIFTING ARM UP TO PLACE BLOCK WILL REQUIRE ENCODER VALUES.
         */
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        lifter = hardwareMap.get(DcMotor.class, "lifter");


        extender = hardwareMap.get(CRServo.class, "extender");
        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(Servo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");


        telemetry.addData("Status:", "Running");
        telemetry.update();

        //set zero power behavior to brake stops the motors from moving at the beginning
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //stop and reset encoder sets the values back to zero upon restarting the robot (or is it initialization? i forget)
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        telemetry.update();

        grabber.setPosition(1); // start in open position


        waitForStart();

        //am also not sure what this does, but it was in the sample code lolol
        //am guessing that it serves a similar purpose to intiializing the robot? don't quote me because i don't actually know :/
        skyStones.activate();


        /**
         * actual movement code
         * go forward, detect, strafe until detected;
         * lift (go forward a bit more?)
         * rotate grabby
         * extend CRservo arm
         *
         */
        int strafeCounter = 0;

        moveStraight(1.15, true);
        while(strafeCounter < 2){ //&& !skyStoneFinder.isVisible()){
            telemetry.addData("found? ", skyStoneFinder.isVisible());
            telemetry.update();
            if(!skyStoneFinder.isVisible()) {
                moveStrafe(.333, true);

                sleep(500);
            }
            strafeCounter += 1;
        }
        moveStrafe(.333, false);
        lift(liftTarget, true); //move up to go over blocc

        /**
         * THIS IS EITHER 0.5 OR 1, IDK WHICH, check with julian for rotator
         */

        //moves for 1s
        extender.setPower(1); //crservo thanks julian
        sleep(6500); // timer for cr servo to rotate

        extender.setPower(0); //stop.
        rotator.setPosition(1); // should set to the right one
        sleep(300);
        //    moveStraight(.2, true);
        lift(liftTarget, false); // move down to grab
        grabber.setPosition(0.1); //closed position
        sleep(300);
        lift(10, true); //lift
        sleep(100);
        //start moving over


        moveStraight(.3,false);
        // moveStraight(.4, false); //move backwards, this value is too high, will crash into other robot as demonstrated match 1
        /**BUT WE HAVE TO BRING IN THE EXTENDER ARM IN ORDER NOT TOCRASH INTO THEM,
         * OR MOVE FORWARD AFTER PASSING THROUGH BRIDGE BEFORE CONTINUING TO STRAFE
         * or else ultraslide go breaky breaky
         */


        moveStrafe( (strafeCounter *  .333) + 3, false);
        //go forward then turn right
        // above (strafing all the way to the foundation) may not be done correctly,
        // originally used a DIRECTION VARIABLE to account for blue and red, but realized that it was disease
        // cuz NEGATIVE VALUES DON'T CHANGE DIRECTION OF MATH.ABS, and manually change true or falses temporarily (ik is bad but not that bad)
        //in the end i will make the true falses color int and only change that, not go and manually true false

        // turn(90,5975, true);
        // moveStrafe(1,false);
        lift(liftTarget, true);
        moveStraight(.3, true); //move up to foundation (value incorrect)

        grabber.setPosition(1); // open position

        rotator.setPosition(0.5);

        extender.setPower(-1);
        sleep(6500);

        moveStraight(.3, false);
        turn(180, 100, false);
        sleep(1500);

//this is foundation pulling, but needs to implement a 360 turn before doing this. turn is sand, might instead just grab second blocc
        // robot moves slowly for precision- want to change code to accomodate for fast boi later on
        //everything after here is possibly incorrect
        //    rightPlatform.setPosition(0.4);
        //    leftPlatform.setPosition(0.4);
        //    moveStraight(1.1, true);

        //  rightPlatform.setPosition(0);
        // leftPlatform.setPosition(0);

        // moveStrafe(1.5, false);

    }

    public void moveStraight(double tiles, boolean forward){ //tileval is encoder val for one tile

        //straight line motion forwards nad backwards, uses frontLft and ackRight
        //USE FRONT RIGHT AND BACK LEFT VALUES FOR STRAFING (trials averaged for a tile strafing right)

        double tilesWithTolerance = (tiles * forwardTarget) - 10;
        int direction = 1;

        if(!forward){
            direction = -1;
        } //forwards vs backwards changes motor direction

        //incremented power is designed to accomodate for acceleration: can probably make it faster
        //i used .0003 to be safe mad not go over 1. can max and min to fix
        double incrementedPower = direction *(forwardTarget -
                (frontLeft.getCurrentPosition() + backRight.getCurrentPosition()) * .5) //slightly sus. changed from /2 to *.5
                *.0003 + .03;
//fix speed
        //while opmode is active() designed to prevent robot moving when stop is pressedd( this has indeed happened before)
        while (opModeIsActive() &&
                (Math.abs(frontLeft.getCurrentPosition()) <= tilesWithTolerance)
                && (Math.abs(backRight.getCurrentPosition()) <= tilesWithTolerance)) {
            frontLeft.setPower(-incrementedPower);
            backLeft.setPower(-incrementedPower);
            frontRight.setPower(incrementedPower);
            backRight.setPower(incrementedPower);


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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                (x.getCurrentPosition() + y.getCurrentPosition()) *.5) //slightly sus, changed from /2
                *.0003 + .03;


        while (opModeIsActive() &&
                (Math.abs(x.getCurrentPosition()) <= tilesWithTolerance)
                && (Math.abs(y.getCurrentPosition()) <= tilesWithTolerance)) {
            frontLeft.setPower(-incrementedPower);
            backLeft.setPower(incrementedPower);
            frontRight.setPower(-incrementedPower);
            backRight.setPower(incrementedPower);


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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void turn(double degrees, double turnDistance, boolean counterclockwise){ //tileval is encoder val for one tile
        int dir = 1;
        if(!counterclockwise){
            dir = -1; //ccw = all negative
        }
        //straight line motion forwards nad backwards, uses frontLft and ackRight
        //USE FRONT RIGHT AND BACK LEFT VALUES FOR STRAFING (trials averaged for a tile strafing right)
        DcMotor x = frontRight;
        DcMotor y = backLeft;
        double percentTurn = degrees / 360;
        double encoderTurnWithTolerance = (percentTurn * turnDistance) - 10;

        double incrementedPower = dir * (strafeTarget -
                (x.getCurrentPosition() + y.getCurrentPosition()) *.5) //slightly sus, fixed
                *.0003 + .03;


        while (opModeIsActive() &&
                (Math.abs(x.getCurrentPosition()) <= encoderTurnWithTolerance)
                && (Math.abs(y.getCurrentPosition()) <= encoderTurnWithTolerance)) {
            frontLeft.setPower(incrementedPower);
            backLeft.setPower(incrementedPower);
            frontRight.setPower(incrementedPower);
            backRight.setPower(incrementedPower);


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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //instructs the motor to send encoder values
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void lift(double target, boolean up){
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int dir = -1;
        if(up){
            dir = 1;
        }
        while(opModeIsActive() && Math.abs(lifter.getCurrentPosition()) < target){
            lifter.setPower(dir * 0.5);
        }
        lifter.setPower(0);

    }
}
