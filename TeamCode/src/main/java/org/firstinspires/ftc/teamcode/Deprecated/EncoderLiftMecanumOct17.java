package org.firstinspires.ftc.teamcode.Deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
@Disabled
public class EncoderLiftMecanumOct17 extends LinearOpMode{

    //create objects- motors, sensors, servos, etc
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor lift;

/** temporary encoder lifting*/
    private int liftermax;

    @Override
    public void runOpMode(){

// define and get the objects using the variable names in the configuration

        frontLeft= hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");
        backLeft= hardwareMap.get(DcMotor.class, "backLeft");
        backRight= hardwareMap.get(DcMotor.class, "backRight");

        lift = hardwareMap.get(DcMotor.class, "lift");

/** LIFT MAX IS SUPPOSED TO SET A STOP USING ENCODERS; LATER WILL USE MAGNETIC LIMIT SWITCH, NEED TO GET ENCODER VALUES FOR THE MAX */
        liftermax = 1000;

   //telemetry is a print statement to the phone
        telemetry.addData("Status:", "Running");
        telemetry.update();

   // sets the 'default' to non-moving
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

   //resets encoder value for lifter AND SETS A LIMIT FOR THE MOTOR USING ENCODERS
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

/** not sure if this is right      lift.setTargetPosition(liftermax); */

   //    pretty sure this is time between init and play
        waitForStart();

   //while play is true on the driver station (loops until stop)

        while(opModeIsActive()){
/**FOR NOW THIS USES 'ENCODER LIFT,' WILL REPLACE WITH LIMIT SWITCH LIFT LATER*/
            move(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y, this.gamepad1.right_stick_x, this.gamepad1.right_stick_y);
            encoderlift(this.gamepad1.dpad_up, this.gamepad1.dpad_down, liftermax, lift.getCurrentPosition());

   //telemetry is print statement to the phone- prints values from encoders
            //telemetry.addData("LIFTER EXTENSION VAL: ");

            telemetry.addData("-----POWER------", "");
            telemetry.addData("Front Left:", frontLeft.getPower());
            telemetry.addData("Front Right:", frontRight.getPower());
            telemetry.addData("Back Left:", backLeft.getPower());
            telemetry.addData("Back Right:", backRight.getPower());


            telemetry.addData("", "");
            telemetry.addData("Lifter: ", lift.getPower());
            telemetry.addData("", "");

            telemetry.addData("LIFTER ENCODER: ", lift.getCurrentPosition());

           /* OTHER TELEMETRY THINGS- ENCODER AND PORT
            telemetry.addData("-----ENCODERS------", "");
            telemetry.addData("Front Left:", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right:", frontRight.getCurrentPosition());
            telemetry.addData("Back Left:", backLeft.getCurrentPosition());
            telemetry.addData("Back Right:", backRight.getCurrentPosition());

            telemetry.addData("", "");
            telemetry.addData("Lifter: ", lift.getCurrentPosition());
            telemetry.addData("", "");


            telemetry.addData("-----PORT------", "");
            telemetry.addData("Front Left:", frontLeft.getPortNumber());
            telemetry.addData("Front Right:", frontRight.getPortNumber());
            telemetry.addData("Back Left:", backLeft.getPortNumber());
            telemetry.addData("Back Right:", backRight.getPortNumber());

            telemetry.addData("", "");
            telemetry.addData("Lifter: ", lift.getPortNumber());
            telemetry.addData("", "");

            */


            telemetry.update();
        }
    }

    public void encoderlift(boolean up, boolean down, int max, int encoderval) {
            int result = 0;
            if (up && encoderval < max) {
//                .6 is to account for super fast boi, can increase to 1 later if needed or add toggle if needed
                lift.setPower(.4);
            } else if (down && encoderval > 0) {
                lift.setPower(-.4);
            } else {
                lift.setPower(0);
            }
        }

    public void move(double leftx, double lefty, double rightx, double righty){

            //straight line motion forwards nad backwards
            if (Math.abs(lefty) > Math.abs(leftx)) {
                frontLeft.setPower(lefty);
                backLeft.setPower(lefty);
                frontRight.setPower(-lefty);
                backRight.setPower(-lefty);
            } else if (Math.abs(leftx) > Math.abs(lefty)) { // strafing

                backLeft.setPower(leftx); //default negative
                frontRight.setPower(-leftx);

                backRight.setPower(leftx);
                frontLeft.setPower(-leftx);

            } else if (Math.abs(rightx) > Math.abs(righty)) {
                frontLeft.setPower(rightx);
                backLeft.setPower(rightx);
                frontRight.setPower(rightx);
                backRight.setPower(rightx);
            }
            if (Math.abs(lefty) < .2 && Math.abs(leftx) < .2) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }

            /*
    THIS IS HOW THE MECANUM WHEELS WORK

    forward: all forward
    backward: all backward
    strafe:  left: fr bl forward
                    fl br backward
             right: fl br forward
                    fr bl backward
    turn: like tank drive; right: left side forward, right side backward
                           left: right side forward, left side backward
    */
        }
}
