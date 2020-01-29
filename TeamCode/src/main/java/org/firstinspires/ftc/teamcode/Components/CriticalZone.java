package org.firstinspires.ftc.teamcode.Components;

public class CriticalZone {

    //Right = +x, left = -x
    private double[] topRightCorner;
    private double[] topLeftCorner;
    private double[] bottomLeftCorner;
    private double[] bottomRightCorner;

    public CriticalZone(double topRightCornerX, double topRightCornerY, double bottomLeftCornerX, double bottomLeftCornerY) {
        this.topRightCorner = new double[] {topRightCornerX, topRightCornerY};
        this.topLeftCorner = new double[] {bottomLeftCornerX, topRightCornerY};
        this.bottomLeftCorner = new double[] {bottomLeftCornerX, bottomLeftCornerY};
        this.bottomRightCorner = new double[] {topRightCornerX, bottomLeftCornerY};
    }

    public CriticalZone(double[] topRightCorner, double[] bottomLeftCorner) {
        this.topRightCorner = topRightCorner;
        this.topLeftCorner = new double[] {bottomLeftCorner[0], topRightCorner[1]};
        this.bottomLeftCorner = bottomLeftCorner;
        this.bottomRightCorner = new double[] {topRightCorner[0], bottomLeftCorner[1]};
    }

    public CriticalZone(double[] topRightCorner, double[] topLeftCorner, double[] bottomRightCorner, double[] bottomLeftCorner) {
        this.topRightCorner = topRightCorner;
        this.topLeftCorner = topLeftCorner;
        this.bottomLeftCorner = bottomLeftCorner;
        this.bottomRightCorner = bottomRightCorner;
    }

    public boolean inCriticalZone(double x, double y) {

        boolean aboveBottom = y > ((bottomLeftCorner[1] - bottomRightCorner[1]) / (bottomLeftCorner[0] - bottomRightCorner[0])) * (x - bottomLeftCorner[0])
                + bottomLeftCorner[1];
        boolean belowTop = y < ((topLeftCorner[1] - topRightCorner[1]) / (topLeftCorner[0] - topRightCorner[0])) * (x - topLeftCorner[0])
                + topLeftCorner[1];
        boolean rightOfLeftSide;
        if(topLeftCorner[0] == bottomLeftCorner[0]) {
            rightOfLeftSide = x > topLeftCorner[0];
        } else {
            rightOfLeftSide = y > ((bottomLeftCorner[1] - topLeftCorner[1]) / (bottomLeftCorner[0] - topLeftCorner[0])) * (x - bottomLeftCorner[0])
                    + bottomLeftCorner[1];
        }
        boolean leftOfRightSide;
        if(topRightCorner[0] == bottomRightCorner[0]) {
            leftOfRightSide = x < topRightCorner[0];
        } else {
            leftOfRightSide = y > ((bottomRightCorner[1] - topRightCorner[1]) / (bottomRightCorner[0] - topRightCorner[0])) * (x - bottomRightCorner[0])
                    + bottomRightCorner[1];
        }

        return aboveBottom && belowTop && leftOfRightSide && rightOfLeftSide;

    }

}
