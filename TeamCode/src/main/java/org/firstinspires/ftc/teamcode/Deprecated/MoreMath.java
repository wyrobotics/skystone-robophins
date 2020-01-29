package org.firstinspires.ftc.teamcode.Deprecated;
import java.lang.Math;

public class MoreMath {

    static double po4 = Math.PI / 4;

    static double norm(double[] vector) {
        double val = 0;
        for(int i = 0; i < vector.length; i++) {
            val = val + vector[i] * vector[i];
        }
        return Math.sqrt(val);
    }

    static double cot(double theta) {
        if ((theta == Math.PI / 2) || (theta == -Math.PI / 2)) {
            return 0;
        } else {
            return 1 / Math.tan(theta);
        }
    }

    static double[] squareProject(double theta) {
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

}
