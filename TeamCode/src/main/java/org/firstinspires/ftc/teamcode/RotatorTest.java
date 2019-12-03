package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RotatorTest extends LinearOpMode {
    private Servo rotator;
    @Override
    public void runOpMode(){
        rotator = hardwareMap.get(Servo.class, "rotator");


        waitForStart();
        while (opModeIsActive()) {
            boolean rightTrigger = false;
            boolean leftTrigger = false;
            double rotatorInc = 0;

            if(!(!rightTrigger ^ (0 != this.gamepad2.right_trigger))) {
                rightTrigger = !rightTrigger;
            }
            if(!(!leftTrigger ^ (0 != this.gamepad1.left_trigger))) {
                leftTrigger = !leftTrigger;
            }
            if(rightTrigger && !leftTrigger) {
                rotatorInc = 0.002;
            } else if(leftTrigger) {
                rotatorInc = -0.002;
            }
            rotator.setPosition(Math.max(0, Math.min(1, rotator.getPosition() + rotatorInc)));

            telemetry.addData("Servo pos: ", rotator.getPosition());
            telemetry.update();
        }
    }
}
