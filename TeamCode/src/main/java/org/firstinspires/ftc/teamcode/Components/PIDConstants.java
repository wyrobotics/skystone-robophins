package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDConstants {

    public static double Kp = 0.015;
    public static double Ki = 0.0000051;
    public static double Kd = 0.00031;
    public static double timeout = 2000;

    public static double moveRelativeP = 40;
    public static double headingCorrection = 1.5;
    public static double accelTime = 1000;
    public static double mrTimeout = 5000;

    public static double strafeP = 0.1;
    public static double strafeYCorrection = 0.2;
    public static double strafeRotateCorrection = 2;

}
