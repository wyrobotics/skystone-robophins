package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.Components.ExtraMath.makeUnit;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.norm;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.rotateVec;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.scalarMult;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.vectorSub;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.headingCorrection;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.moveRelativeP;

public abstract class AutonomousOpMode extends LinearOpMode {

    protected AutonomousRobot autonomousRobot;

    protected void moveRelative(double deltaX, double deltaY) {

        double[] initState = autonomousRobot.odometryTracker.getPosition();


        //Real target on plane
        double[] targetPos = new double[] {initState[0] + (deltaX * Math.cos(initState[2])) - (deltaY * Math.sin(initState[2])),
                initState[1] + (deltaX * Math.sin(initState[2])) + (deltaY * Math.cos(initState[2]))};

        //Robot's target (displacement vector)
        double[] displacement = new double[] {deltaX, deltaY};
        double[] movementVec = makeUnit(displacement);
        double[] drivePowers;
        double[] currentPos = initState;

        while(norm(displacement) > 0.2 && opModeIsActive()) {

            double angularError = initState[2] - currentPos[2];

            drivePowers = scalarMult(autonomousRobot.driveBase.stickToMotorPowers(movementVec[0], movementVec[1]),
                    Math.min(1,norm(displacement) / moveRelativeP));
            drivePowers[0] = drivePowers[0] - (headingCorrection * angularError);
            drivePowers[1] = drivePowers[1] + (headingCorrection * angularError);
            drivePowers[2] = drivePowers[2] - (headingCorrection * angularError);
            drivePowers[3] = drivePowers[3] + (headingCorrection * angularError);
            autonomousRobot.driveBase.setMotorPowers(drivePowers);

            currentPos = autonomousRobot.odometryTracker.getPosition();
            displacement = rotateVec(vectorSub(targetPos,new double[] {currentPos[0], currentPos[1]}), -currentPos[2]);
            movementVec = makeUnit(displacement);


            telemetry.addData("Displacement x: ", displacement[0]);
            telemetry.addData("Displacement y: ", displacement[1]);
            telemetry.addData("Target x: ", targetPos[0]);
            telemetry.addData("Target y: ", targetPos[1]);
            telemetry.addData("Current x: ", currentPos[0]);
            telemetry.addData("Current y: ", currentPos[1]);
            telemetry.update();


        }

    }

    public void moveStraight(double distance, boolean forward) {

        double[] initPos = autonomousRobot.odometryTracker.getPosition();

    }

}
