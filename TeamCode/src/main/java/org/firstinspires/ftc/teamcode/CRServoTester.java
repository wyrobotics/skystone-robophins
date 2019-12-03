package org.firstinspires.ftc.teamcode;

public class CRServoTester {
    /**package org.firstinspires.ftc.teamcode;

     import com.qualcomm.robotcore.eventloop.opmode.Disabled;
     import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
     import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
     import com.qualcomm.robotcore.hardware.CRServo;

     @TeleOp
     @Disabled
     public class CRServoTester extends LinearOpMode{

     //create objects- motors, sensors, servos, etc
     private CRServo leftGrab;
     private CRServo rightGrab;


     @Override
     public void runOpMode(){

     // define and get the objects using the variable names in the configuration

     leftGrab = hardwareMap.get(CRServo.class, "leftPlatform");
     rightGrab = hardwareMap.get(CRServo.class, "rightPlatform");




     //try is a print statement to the phone
     telemetry.addData("Status:", "Running");
     telemetry.update();

     // sets the 'default' pos

     /

     //right forward, left reverse;
     leftGrab.setDirection(CRServo.Direction.REVERSE);
     rightGrab.setDirection(CRServo.Direction.FORWARD);

     leftGrab.setPosition(.25);
     rightGrab.setPosition(.25);
     //resets encoder value for lifter AND SETS A LIMIT FOR THE MOTOR USING ENCODERS


     /** not sure if this is right      lift.setTargetPosition(liftermax); */

    //    pretty sure this is time between init and play
   // waitForStart();

    //while play is true on the driver station (loops until stop)

    /*    while(opModeIsActive()){
        boolean moved = false;
        if(this.gamepad1.a) {
            leftGrab.setPosition(.5);
            rightGrab.setPosition(.5);
            moved = true;
        }
        telemetry.addData("moved? ", moved);
        telemetry.update();
    }
}
}
        */
}
