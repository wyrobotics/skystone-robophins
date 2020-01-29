package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;

import static org.firstinspires.ftc.teamcode.Components.PIDConstants.Kd;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.Ki;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.Kp;


@TeleOp
public class RotationPID extends LinearOpMode {

    //Drivebase motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Arm/hand stuff
    private DcMotor lifter;
    private CRServo extender;
    private DigitalChannel extenderSwitch;
    private Servo rotator;
    private CRServo grabber;

    //Plate movers, names based on facing robot from behind
    private Servo leftPlatform;
    private Servo rightPlatform;


    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;


    public double cot(double theta) {
        if ((theta == Math.PI / 2) || (theta == -Math.PI / 2)) {
            return 0;
        } else {
            return 1 / Math.tan(theta);
        }
    }

    double po4 = Math.PI / 4;

    private double[] squareProject(double theta) {
        double[] output = new double[2];
        if((theta < po4) && (theta >= -po4)) {
            output[0] = 1;
            output[1] = Math.tan(theta);
        } else if((theta < 3 * po4) && (theta >= po4)) {
            output[0] = cot(theta);
            output[1] = 1;
        } else if((theta >= 3 * po4) || (theta <= -3 * po4)) {
            output[0] = -1;
            output[1] = -Math.tan(theta);
        } else if((theta < -po4) && (theta >= -3 * po4)) {
            output[0] = -cot(theta);
            output[1] = -1;
        }
        return output;
    }

    Boolean aPressed = false;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        lifter = hardwareMap.get(DcMotor.class, "lifter");
        extender = hardwareMap.get(CRServo.class, "extender");
        extenderSwitch = hardwareMap.get(DigitalChannel.class, "extenderLimitSwitch");
        rotator = hardwareMap.get(Servo.class, "rotator");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        leftPlatform = hardwareMap.get(Servo.class, "leftPlatform");
        rightPlatform = hardwareMap.get(Servo.class, "rightPlatform");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftPlatform.setDirection(Servo.Direction.REVERSE);
        rightPlatform.setDirection(Servo.Direction.FORWARD);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotator.setPosition(0.5);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;



        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        boolean pressedYet = false;





        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("gyroscope: ", getAngle());
            if(!(!aPressed ^ this.gamepad1.a)) {
                aPressed = !aPressed;
            }
            if(aPressed && !pressedYet) {
                pressedYet = true;
                targetPosition = getAngle() + 90;
                turn90();
            }

            if(this.gamepad1.b) {
                pressedYet = false;
            }

        }

    }
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        //tried: ZYX, XYZ, YZX, ZXY*,

        double deltaAngle = angles.secondAngle - lastAngles.secondAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;

        lastAngles = angles;

        telemetry.addData("First angle: ", angles.firstAngle);
        telemetry.update();

        return globalAngle;
    }

    double targetPosition = 0;

    //double Kp = 0.03;
    //double Ki = 0.00026;
    //double Kd = 0.0019;

    /*
    double Kp = 0.03;
    double Kd = 0.0019;
    double Ki = 0.0025;

     */

    /*
    double Kp = 0.03;
    double Kd = 0.000007;
    double Ki = 0.0000028;
    Decent ^^^
     */


    //double Kp = 0.03;
    //double Ki = 0.000005;
    //double Kd = 0.000007;
    //double Kd = 0.0003;
    //Kinda ass ^^^

    //ALL OF THE ABOVE ARE kinda bad


    //double Kp = 0.013;
    //double Ki = 0.000015;
    //double Kd = 0.00032;
    //GOOD ^^

    //double Kp = 0.016;
    //double Ki = 0.000015;
    //double Kd = 0.0006;
    //GOOD ^^


    //Trash now; work from here though




    //double Kp = 0.025;
    //double Ki = 0.00001;
    //double Kd = 0.0003;

    private double error() {
        return targetPosition - getAngle();
    }

    private void turn90() {

        double lastError = error();
        double lastTime = System.currentTimeMillis();

        double proportionTerm;
        double derivativeTerm;
        double integralTerm = 0;

        double u = 0;

        while(opModeIsActive() && Math.abs(error()) > 1) {
            double error = error();
            double time = System.currentTimeMillis();
            proportionTerm = Kp * error;
            derivativeTerm = Kd * ((error - lastError) / Math.abs((lastTime - time)));
            integralTerm += Ki * error * Math.abs((lastTime - time));

            u = proportionTerm + integralTerm + derivativeTerm;

            frontLeft.setPower(u);
            frontRight.setPower(-u);
            backLeft.setPower(u);
            backRight.setPower(-u);

            telemetry.addData("error: ", lastError);
            telemetry.addData("front left: ", frontLeft.getPower());
            telemetry.addData("frontRight: ", frontRight.getPower());
            telemetry.addData("back left: ", backLeft.getPower());
            telemetry.addData("back right: ", backRight.getPower());

            telemetry.update();

            lastError = error;
            lastTime = time;
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }







}
