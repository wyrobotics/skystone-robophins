package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        MainRobot mainRobot = new MainRobot(hardwareMap, telemetry, 0.75);

        waitForStart();

        while(opModeIsActive()) {

            mainRobot.driveBase.teleOpMove(this.gamepad1.left_stick_x,-this.gamepad1.left_stick_y,this.gamepad1.right_stick_x);

        }

    }

}
