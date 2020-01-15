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

    private ExecutorService odometryUpdaterExecutor = ThreadPool.newSingleThreadExecutor("Odometry Updater");
    private Boolean continueExecution = true;

    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor normalEncoder;

    private double leftValue;
    private double rightValue;
    private double normalValue;

    private Telemetry telemetry;

    private double xPos;
    private double yPos;
    private double theta;

    private double podDistance = 17;

    public OdometryTracker(HardwareMap hardwareMap, Telemetry telemetry) {

        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        normalEncoder = hardwareMap.get(DcMotor.class, "normalEncoder");

        this.telemetry = telemetry;

    }

    private void updatePosition() {

        double l = leftEncoder.getCurrentPosition(),
                r = rightEncoder.getCurrentPosition(),
                n = normalEncoder.getCurrentPosition();

        double deltaL = l - leftValue,
                deltaR = r - rightValue,
                deltaN = n - normalValue,
                deltaF = (deltaL + deltaR)/2;

        xPos += deltaN * Math.cos(theta);
        yPos += deltaN * Math.sin(theta);

        xPos += -deltaF * Math.sin(theta);
        yPos += deltaF * Math.cos(theta);

        theta += (deltaR - deltaF) / podDistance;

    }

    private Runnable odometryUpdaterRunnable = new Runnable() {
        @Override
        public void run() {
            while(continueExecution) {
                updatePosition();
            }
        }
    };

    public void startOdometry() {
        odometryUpdaterExecutor.execute(odometryUpdaterRunnable);
    }

    public void shutdownOdometry() {
        continueExecution = false;
        odometryUpdaterExecutor.shutdown();
    }

    public double[] getPosition() {
        double[] pos = {xPos, yPos, theta};
        return pos;
    }

}
