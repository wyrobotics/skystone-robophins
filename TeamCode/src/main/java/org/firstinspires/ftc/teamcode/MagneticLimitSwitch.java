package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class MagneticLimitSwitch extends LinearOpMode {
    private DigitalChannel limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("limit found? ", limitSwitch.getState());
            telemetry.addData("'mode' ", limitSwitch.getMode());
            telemetry.update();
        }
    }

}
