package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoTester extends LinearOpMode{

    //create objects- motors, sensors, servos, etc
    private Servo leftGrab;
    private Servo rightGrab;


    @Override
    public void runOpMode(){

// define and get the objects using the variable names in the configuration

        leftGrab = hardwareMap.get(Servo.class, "leftPlatform");
        rightGrab = hardwareMap.get(Servo.class, "rightPlatform");




    //try is a print statement to the phone
        telemetry.addData("Status:", "Running");
        telemetry.update();

        // sets the 'default' pos


        //right forward, left reverse;
        leftGrab.setDirection(Servo.Direction.REVERSE);
        rightGrab.setDirection(Servo.Direction.FORWARD);

        int lpos = 0;
        int rpos = 0;

        leftGrab.setPosition(lpos);
        rightGrab.setPosition(rpos);
        //resets encoder value for lifter AND SETS A LIMIT FOR THE MOTOR USING ENCODERS


/** not sure if this is right      lift.setTargetPosition(liftermax); */

        //    pretty sure this is time between init and play
        waitForStart();

        //while play is true on the driver station (loops until stop)

        double moved = -.4;

        while(opModeIsActive()) {
            if(this.gamepad1.a) {
                moved = moved * -1;
            }

            leftGrab.setPosition(lpos + moved);
            rightGrab.setPosition(rpos + moved);

            telemetry.addData("left = ", lpos + moved);
            telemetry.addData("right = ", rpos + moved);
            telemetry.addData("moved? ", moved);
            telemetry.update();
        }
     /*   boolean moved = false;

        while(opModeIsActive()){
            if (this.gamepad1.a) {
                if (!moved) {
                    leftGrab.setPosition(.5);
                    rightGrab.setPosition(-.5);
                    moved = true;
                } else {
                    leftGrab.setPosition(0);
                    rightGrab.setPosition(0);
                    moved = false;
                }
            }*/

    }
}
