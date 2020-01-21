package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.Math;
import java.util.concurrent.ExecutorService;

public class OdometryTracker {

    private ExecutorService odometryUpdaterExecutor;
    private Boolean continueExecution = true;

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor normalEncoder;

    private double leftValue;
    private double rightValue;
    private double normalValue;

    private Telemetry telemetry;

    private double xPos = 0;
    private double yPos = 0;
    private double theta = 0;

    //Dummy values right now, determine both experimentally
    private double podDistance = 17;
    private double countsPerInch = 1250;

    public OdometryTracker(HardwareMap hardwareMap, Telemetry telemetry) {

        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        normalEncoder = hardwareMap.get(DcMotor.class, "shooter");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        normalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        normalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.telemetry = telemetry;

    }

    private void updatePosition() {

        double l = leftEncoder.getCurrentPosition() / countsPerInch,
                r = rightEncoder.getCurrentPosition() / countsPerInch,
                n = normalEncoder.getCurrentPosition() / countsPerInch;

        double deltaL = l - leftValue,
                deltaR = r - rightValue,
                deltaN = n - normalValue,
                deltaF = (deltaL + deltaR)/2;

        xPos += deltaN * Math.cos(theta);
        yPos += deltaN * Math.sin(theta);

        xPos += -deltaF * Math.sin(theta);
        yPos += deltaF * Math.cos(theta);

        theta += (deltaR - deltaF) / podDistance;

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
            }
        }
    };

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

}
