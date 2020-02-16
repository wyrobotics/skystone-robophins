package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;
import java.util.concurrent.ExecutorService;

import static org.firstinspires.ftc.teamcode.Components.OdometryConfig.countsPerInch;
import static org.firstinspires.ftc.teamcode.Components.OdometryConfig.podDistance;

public class OdometryTracker {

    private ExecutorService odometryUpdaterExecutor;
    private Boolean continueExecution = true;

    public DcMotor leftEncoder;
    public DcMotor rightEncoder;
    public DcMotor normalEncoder;

    private double leftValue;
    private double rightValue;
    private double normalValue;

    private Telemetry telemetry;


    private double xPos = 0;
    private double yPos = 0;
    private double theta = 0;

    private double odomTheta = 0;

    public OdometryTracker(HardwareMap hardwareMap, Telemetry telemetry) {

        leftEncoder = hardwareMap.get(DcMotor.class, "backLifter");
        rightEncoder = hardwareMap.get(DcMotor.class, "backRotator");
        normalEncoder = hardwareMap.get(DcMotor.class, "shooter");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        normalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        normalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;

    }

    private void updatePosition() {

        double l = -leftEncoder.getCurrentPosition() / countsPerInch,
                r = -rightEncoder.getCurrentPosition() / countsPerInch,
                n = -normalEncoder.getCurrentPosition() / countsPerInch;

        double deltaL = l - leftValue,
                deltaR = r - rightValue,
                deltaN = n - normalValue,
                deltaF = (deltaL + deltaR)/2;

        xPos += -deltaN * Math.cos(theta);
        yPos += -deltaN * Math.sin(theta);

        xPos += -deltaF * Math.sin(theta);
        yPos += deltaF * Math.cos(theta);

        theta += (deltaR - deltaL) / podDistance;
        //theta = getAngle();

        leftValue = l;
        rightValue = r;
        normalValue = n;

    }

    private Runnable odometryUpdaterRunnable = new Runnable() {
        @Override
        public void run() {
            continueExecution = true;

            leftValue = leftEncoder.getCurrentPosition() / countsPerInch;
            rightValue = rightEncoder.getCurrentPosition() / countsPerInch;
            normalValue = normalEncoder.getCurrentPosition() / countsPerInch;

            while (continueExecution && !Thread.currentThread().isInterrupted()) {
                updatePosition();
                sleep(10);
            }
        }
    };

    private void sleep(double milliseconds) {

        double time = System.currentTimeMillis();

        while (System.currentTimeMillis() - time < milliseconds) { }

    }

    public void startOdometry() {
        odometryUpdaterExecutor = ThreadPool.newSingleThreadExecutor("Odometry Updater");
        odometryUpdaterExecutor.execute(odometryUpdaterRunnable);
    }

    public void shutdownOdometry() {
        continueExecution = false;
        odometryUpdaterExecutor.shutdownNow();
        odometryUpdaterExecutor = null;
    }

    public double[] getPosition() {
        double[] pos = {xPos, yPos, theta};
        return pos;
    }

    public double getAngle() { return 69; }

    public double getOdomAngle() { return odomTheta; }

}
