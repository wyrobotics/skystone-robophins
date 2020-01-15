package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TapeShooter extends LinearOpMode {

    DcMotor shooter;

    public void runOpMode() {

        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {

            shooter.setPower(-this.gamepad1.left_stick_y);

        }

    }

}
