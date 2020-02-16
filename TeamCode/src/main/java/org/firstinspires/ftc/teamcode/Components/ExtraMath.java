package org.firstinspires.ftc.teamcode.Components;

public class ExtraMath {

    public static double cot(double theta) {
        if ((theta == Math.PI / 2) || (theta == -Math.PI / 2)) {
            return 0;
        } else {
            return 1 / Math.tan(theta);
        }
    }

    public static double po4 = Math.PI / 4;

    public static double[] squareProject(double theta) {
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

    public static double[] vectorSub(double[] v, double[] u) {
        double[] difference = new double[v.length];
        for(int i = 0; i < v.length; i++) {
            difference[i] = v[i] - u[i];
        }
        return difference;
    }

    public static double[] scalarMult(double[] v, double a) {
        double[] u = new double[v.length];
        for(int i = 0; i < v.length; i++) {
            u[i] = v[i] * a;
        }
        return u;
    }

    public static double norm(double[] v) {
        double normSquared = 0;
        for(int i = 0; i < v.length; i++) {
            normSquared += v[i] * v[i];
        }
        return Math.sqrt(normSquared);
    }

    public static double[] makeUnit(double[] v) { return scalarMult(v, 1 / norm(v)); }

    public static double[] rotateVec(double[] v, double theta) {
        return new double[] {(v[0] * Math.cos(theta)) - (v[1] * Math.sin(theta)), (v[0] * Math.sin(theta)) + (v[1] * Math.cos(theta))};
    }

    public static double range(double val, double lower, double upper) {
        return Math.max(lower,Math.min(upper,val));
    }

}
