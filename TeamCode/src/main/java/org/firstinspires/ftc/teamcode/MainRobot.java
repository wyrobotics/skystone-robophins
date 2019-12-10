package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.MoreMath.cot;

public class MainRobot {

    public DriveBase driveBase;

    public DcMotor lifter;
    public CRServo extender;

    private Servo rotator;
    private CRServo grabber;

    private Servo leftPlatform;
    private Servo rightPlatform;

    private Servo leftSlapper;
    private Servo rightSlapper;

    private DigitalChannel extenderSwitch;
    private DigitalChannel lifterSwitch;

    public MainRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        driveBase = new DriveBase(hardwareMap, telemetry);

        lifter = hardwareMap.get(DcMotor.class, "lifter");
        extender = hardwareMap.get(CRServo.class, "extender");

        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        leftSlapper = hardwareMap.get(Servo.class, "leftSlapper");
        rightSlapper = hardwareMap.get(Servo.class, "rightSlapper");

        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        lifterSwitch = hardwareMap.get(DigitalChannel.class, "lifterLimitSwitch");

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(0.5);

    }

}
