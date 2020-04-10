package org.firstinspires.ftc.teamcode.Components;

public class Position {

    private static double xPos, yPos, theta;

    public static double getX() {return xPos;}
    public static double getY() {return yPos;}
    public static double getTheta() {return theta;}

    public static void setX(double pos) {xPos = pos;}
    public static void setY(double pos) {yPos = pos;}
    public static void setTheta(double pos) {theta = pos;}

    public static void setPos(double x, double y, double t) {
        xPos = x;
        yPos = y;
        theta = t;
    }

}
