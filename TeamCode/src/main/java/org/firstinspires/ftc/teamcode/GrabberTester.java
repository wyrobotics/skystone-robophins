package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class GrabberTester extends LinearOpMode{

    //create objects- motors, sensors, servos, etc
    private Servo grabber;

    @Override
    public void runOpMode(){

// define and get the objects using the variable names in the configuration

        grabber = hardwareMap.get(Servo.class, "grabber");


        //try is a print statement to the phone
        telemetry.addData("Status:", "Running");
        telemetry.update();

        // sets the 'default' pos


        //right forward, left reverse;
     //grabber.setDirection(Servo.Direction.REVERSE);
        double pos = .25;

        grabber.setPosition(pos);

        //    pretty sure this is time between init and play
        waitForStart();

        //while play is true on the driver station (loops until stop)

        double moved = -.6;

        while(opModeIsActive()) {
            if(this.gamepad1.a) {
                moved = moved * -1;
            }

            grabber.setPosition(pos + moved);

            telemetry.addData("pos = ", pos + moved);
            telemetry.addData("moved? ", moved);
            telemetry.update();
        }

    }
}
