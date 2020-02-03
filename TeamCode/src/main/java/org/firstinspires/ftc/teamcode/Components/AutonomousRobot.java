package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.teamcode.Components.ExtraMath.vectorSub;

public class AutonomousRobot extends MainRobot {

    public static final String TAG = "Vuforia VuMark Sample";

    public OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    public AutonomousRobot(HardwareMap hardwareMap, Telemetry telemetry, double speed) {

        super(hardwareMap, telemetry, speed);



    }

}
