package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Extender implements Runnable {

    CRServo extender;
    DigitalChannel extenderLimitSwitch;

    double extenderPower;
    double extendTime;

    public Extender(CRServo extender, DigitalChannel extenderLimitSwitch) {
        this.extender = extender;
        this.extenderLimitSwitch = extenderLimitSwitch;
    }

    public void extend(double power) {

    }
    public void extend(double power, double time) {
        extenderPower = power;
        extendTime = time;
        run();
    }

    public void run() {
        double startTime = System.currentTimeMillis();
        extender.setPower(1);
    }

}
