package org.firstinspires.ftc.teamcode;
import java.lang.Math;

public class VecMath {

    public double norm(double[] vector) {
        double val = 0;
        for(int i = 0; i < vector.length; i++) {
            val = val + vector[i] * vector[i];
        }
        return Math.sqrt(val);
    }

}
