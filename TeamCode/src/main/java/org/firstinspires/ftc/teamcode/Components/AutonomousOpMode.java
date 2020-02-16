package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import static org.firstinspires.ftc.teamcode.Components.ExtraMath.makeUnit;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.norm;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.range;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.rotateVec;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.scalarMult;
import static org.firstinspires.ftc.teamcode.Components.ExtraMath.vectorSub;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.Kd;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.Ki;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.Kp;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.accelTime;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.headingCorrection;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.moveRelativeP;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.mrTimeout;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.strafeP;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.strafeRotateCorrection;
import static org.firstinspires.ftc.teamcode.Components.PIDConstants.strafeYCorrection;

public abstract class AutonomousOpMode extends LinearOpMode {

    protected AutonomousRobot autonomousRobot;

    //moveRelative

    protected void moveRelative(double deltaX, double deltaY, double timeout) {

        double[] initState = autonomousRobot.odometryTracker.getPosition();
        double initTime = System.currentTimeMillis();

        //Real target on cartesian plane
        double[] targetPos = new double[]{initState[0] + (deltaX * Math.cos(initState[2])) - (deltaY * Math.sin(initState[2])),
                initState[1] + (deltaX * Math.sin(initState[2])) + (deltaY * Math.cos(initState[2]))};

        //Robot's target (displacement vector)
        double[] displacement = new double[]{deltaX, deltaY};
        double[] movementVec = makeUnit(displacement);
        double[] drivePowers;
        double[] currentPos = initState;
        double angularError = 0;

        while ((norm(displacement) > 0.5 || Math.abs(angularError) > 0.03) && (System.currentTimeMillis() - initTime < timeout) && opModeIsActive()) {

            angularError = initState[2] - currentPos[2];

            drivePowers = scalarMult(autonomousRobot.driveBase.stickToMotorPowers(movementVec[0], movementVec[1]),
                    Math.min(1, norm(displacement) / moveRelativeP) * Math.min(1, (System.currentTimeMillis() - initTime) / accelTime));
            drivePowers[0] = drivePowers[0] - (headingCorrection * angularError);
            drivePowers[1] = drivePowers[1] + (headingCorrection * angularError);
            drivePowers[2] = drivePowers[2] - (headingCorrection * angularError);
            drivePowers[3] = drivePowers[3] + (headingCorrection * angularError);
            autonomousRobot.driveBase.setMotorPowers(drivePowers);

            currentPos = autonomousRobot.odometryTracker.getPosition();
            displacement = rotateVec(vectorSub(targetPos, new double[]{currentPos[0], currentPos[1]}), -currentPos[2]);
            movementVec = makeUnit(displacement);


            telemetry.addData("Displacement x: ", displacement[0]);
            telemetry.addData("Displacement y: ", displacement[1]);
            //telemetry.addData("Target x: ", targetPos[0]);
            //telemetry.addData("Target y: ", targetPos[1]);
            telemetry.addData("Current x: ", currentPos[0]);
            telemetry.addData("Current y: ", currentPos[1]);
            telemetry.addData("Angular error: ", angularError);
            telemetry.addData("Left Encoder: ", autonomousRobot.odometryTracker.leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder: ", autonomousRobot.odometryTracker.rightEncoder.getCurrentPosition());
             /*
            telemetry.addData("Front Left: ", drivePowers[0]);
            telemetry.addData("Front Right: ", drivePowers[1]);
            telemetry.addData("Back Left: ", drivePowers[2]);
            telemetry.addData("Back Right: ", drivePowers[3]);

              */
            telemetry.update();


        }

        autonomousRobot.driveBase.setMotorPowers(0, 0, 0, 0);

    }



    /*
    protected void moveRelative(double deltaX, double deltaY, double dTheta) {

        double deltaTheta = (dTheta / 360) * 2 * Math.PI;

        double[] initState = autonomousRobot.odometryTracker.getPosition();
        double initTime = System.currentTimeMillis();

        //Real target on cartesian plane
        double[] targetPos = new double[] {initState[0] + (deltaX * Math.cos(initState[2])) - (deltaY * Math.sin(initState[2])),
                initState[1] + (deltaX * Math.sin(initState[2])) + (deltaY * Math.cos(initState[2]))};

        //Robot's target (displacement vector)
        double[] displacement = new double[] {deltaX, deltaY};
        double[] movementVec = makeUnit(displacement);
        double[] drivePowers;
        double[] currentPos = initState;
        double angularError = deltaTheta;

        while((norm(displacement) > 0.2 || Math.abs(angularError) > 0.01) && (System.currentTimeMillis() - initTime < mrTimeout) && opModeIsActive()) {

            angularError = initState[2] + deltaTheta - currentPos[2];

            drivePowers = scalarMult(autonomousRobot.driveBase.stickToMotorPowers(movementVec[0], movementVec[1]),
                    Math.min(1,norm(displacement) / moveRelativeP) * Math.min(1, (System.currentTimeMillis() - initTime) / accelTime));
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
            //telemetry.addData("Target x: ", targetPos[0]);
            //telemetry.addData("Target y: ", targetPos[1]);
            telemetry.addData("Current x: ", currentPos[0]);
            telemetry.addData("Current y: ", currentPos[1]);
            telemetry.addData("Angular error: ", angularError);
            telemetry.addData("Left Encoder: ", autonomousRobot.odometryTracker.leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder: ", autonomousRobot.odometryTracker.rightEncoder.getCurrentPosition());

            telemetry.addData("Front Left: ", drivePowers[0]);
            telemetry.addData("Front Right: ", drivePowers[1]);
            telemetry.addData("Back Left: ", drivePowers[2]);
            telemetry.addData("Back Right: ", drivePowers[3]);


            telemetry.update();


        }

        autonomousRobot.driveBase.setMotorPowers(0,0,0,0);

    }

    */

    protected void moveStrafe(double distance, boolean left) {

        double dir = left ? 1 : -1;
        double[] currentPosition = autonomousRobot.odometryTracker.getPosition();
        double initAngle = currentPosition[2];

        double[] targetPosition = new double[]{currentPosition[0] - (dir * distance * Math.cos(currentPosition[2])),
                currentPosition[1] - (dir * distance * Math.sin(currentPosition[2]))};

        double[] displacement = vectorSub(targetPosition, currentPosition);

        double angularError = 0;

        double[] drivePowers = new double[4];
        double dx;

        while (((-dir * displacement[0] > 0.2) && opModeIsActive())) {

            drivePowers[0] = -0.4 * dir - 2 * (headingCorrection * angularError);
            drivePowers[1] = 0.8 * dir + 2 * (headingCorrection * angularError);
            drivePowers[2] = 0.8 * dir - 2 * (headingCorrection * angularError);
            drivePowers[3] = -0.4 * dir + 2 * (headingCorrection * angularError);

            autonomousRobot.driveBase.setMotorPowers(drivePowers);

            currentPosition = autonomousRobot.odometryTracker.getPosition();
            displacement = vectorSub(targetPosition, new double[]{currentPosition[0], currentPosition[1]});

            angularError = initAngle - currentPosition[2];

            telemetry.addData("Displacement x: ", displacement[0]);
            telemetry.addData("Displacement y: ", displacement[1]);
            telemetry.update();

        }

        autonomousRobot.driveBase.setMotorPowers(0, 0, 0, 0);

    }

    protected void turn(double angle, boolean ccw) {

        double dir = ccw ? 1 : -1;
        double currentAngle = autonomousRobot.odometryTracker.getPosition()[2];
        double targetAngle = currentAngle + (dir * (2 * Math.PI * angle / 360));

        if (ccw) {
            while (currentAngle < targetAngle && opModeIsActive()) {
                double u = Math.min(1, Math.abs((targetAngle - currentAngle) / (Math.PI / 2)));
                autonomousRobot.driveBase.setMotorPowers(-u, u, -u, u);
                currentAngle = autonomousRobot.odometryTracker.getPosition()[2];
                telemetry.addData("Current Angle: ", currentAngle);
                telemetry.addData("Target Angle: ", targetAngle);
                telemetry.update();
            }
        } else {
            while (currentAngle > targetAngle && opModeIsActive()) {
                double u = Math.min(1, Math.abs((targetAngle - currentAngle) / (Math.PI / 2)));
                autonomousRobot.driveBase.setMotorPowers(u, -u, u, -u);
                currentAngle = autonomousRobot.odometryTracker.getPosition()[2];
            }
        }

        autonomousRobot.driveBase.setMotorPowers(0, 0, 0, 0);

    }

    protected void turnTimeout(double angle, boolean ccw, double timeout) {

        double dir = ccw ? 1 : -1;
        double currentAngle = autonomousRobot.odometryTracker.getPosition()[2];
        double targetAngle = currentAngle + (dir * (2 * Math.PI * angle / 360));
        double initTime = System.currentTimeMillis();

        if (ccw) {
            while (currentAngle < targetAngle && opModeIsActive() && (initTime > System.currentTimeMillis() - timeout)) {
                double u = Math.min(1, Math.abs((targetAngle - currentAngle) / (Math.PI / 2)));
                autonomousRobot.driveBase.setMotorPowers(-u, u, -u, u);
                currentAngle = autonomousRobot.odometryTracker.getPosition()[2];
                telemetry.addData("Current Angle: ", currentAngle);
                telemetry.addData("Target Angle: ", targetAngle);
                telemetry.update();
            }
        } else {
            while (currentAngle > targetAngle && opModeIsActive()) {
                double u = Math.min(1, Math.abs((targetAngle - currentAngle) / (Math.PI / 2)));
                autonomousRobot.driveBase.setMotorPowers(u, -u, u, -u);
                currentAngle = autonomousRobot.odometryTracker.getPosition()[2];
            }
        }

        autonomousRobot.driveBase.setMotorPowers(0, 0, 0, 0);

    }

    protected void turnTo(double angle, boolean ccw, double timeout) {

        double dir = ccw ? 1 : -1;
        double currentAngle = autonomousRobot.odometryTracker.getPosition()[2];
        double targetAngle = 2 * Math.PI * angle / 360;
        double initTime = System.currentTimeMillis();

        if (ccw) {
            while (currentAngle < targetAngle && opModeIsActive() && (initTime > System.currentTimeMillis() - timeout)) {
                double u = Math.min(1, Math.abs((targetAngle - currentAngle) / (Math.PI / 2)));
                autonomousRobot.driveBase.setMotorPowers(-u, u, -u, u);
                currentAngle = autonomousRobot.odometryTracker.getAngle();
                telemetry.addData("Current Angle: ", currentAngle);
                telemetry.addData("Target Angle: ", targetAngle);
                telemetry.update();
            }
        } else {
            while (currentAngle > targetAngle && opModeIsActive() && (initTime > System.currentTimeMillis() - timeout)) {
                double u = Math.min(1, Math.abs((targetAngle - currentAngle) / (Math.PI / 2)));
                autonomousRobot.driveBase.setMotorPowers(u, -u, u, -u);
                currentAngle = autonomousRobot.odometryTracker.getAngle();
            }
        }

        autonomousRobot.driveBase.setMotorPowers(0, 0, 0, 0);

    }

    //chlo simple backup functions
    //backup encoder values

    protected void simpleTurn(double angle, boolean ccw) {
        double initialpos = 180 * autonomousRobot.odometryTracker.getPosition()[2] / Math.PI;
        int dir = 1;
        if (ccw) {
            //ccw then right positive left negative
            dir = -1;
        }
        while (opModeIsActive() && Math.abs(180 * autonomousRobot.odometryTracker.getPosition()[2] / Math.PI) - angle < 10){
            //autonomousRobot.driveBase.frontLeft.setPower(dir);
            //autonomousRobot.driveBase.backLeft.setPower(dir);
            // autonomousRobot.driveBase.frontRight.setPower(-dir);
            //   autonomousRobot.driveBase.backRight.setPower(-dir);

            telemetry.addData("angle diff: ", Math.abs(180 * autonomousRobot.odometryTracker.getPosition()[2] / Math.PI));// Math.abs(180*(autonomousRobot.odometryTracker.getPosition()[2]) / Math.PI));
            telemetry.addData("inital pos: ", initialpos);
            telemetry.update();
    }
        autonomousRobot.driveBase.frontLeft.setPower(0);
        autonomousRobot.driveBase.backLeft.setPower(0);
        autonomousRobot.driveBase.frontRight.setPower(0);
        autonomousRobot.driveBase.backRight.setPower(0);

    }


    protected void strafeProfiled(double distance, boolean right) {

        double[] initPos = autonomousRobot.odometryTracker.getPosition();

        double dir = right ? 1 : -1;

        double[] dPos = rotateVec(new double[] {distance * dir, 0}, initPos[2]);

        double[] targetPos = new double[] {initPos[0], initPos[1], initPos[2]};
        targetPos[0] += dPos[0];
        targetPos[1] += dPos[1];

        double[] relativeDisplacement = rotateVec(vectorSub(targetPos, initPos), -initPos[2]);

        double thetaError = 0;

        double[] motorPowers = new double[] {0, 0, 0, 0};

        double[] currentPosition;

        double initTime = System.currentTimeMillis();

        while(opModeIsActive() && Math.abs(relativeDisplacement[0]) > 0.5 && System.currentTimeMillis() - 3000 < initTime) {

            motorPowers[0] = dir * range(relativeDisplacement[0],-1,1) * (range(relativeDisplacement[0] * strafeP,-1,1));
            motorPowers[1] = dir * -range(relativeDisplacement[0],-1,1) * (range(relativeDisplacement[0] * strafeP,-1,1));
            motorPowers[2] = dir * -range(relativeDisplacement[0],-1,1) * (range(relativeDisplacement[0] * strafeP,-1,1));
            motorPowers[3] = dir * range(relativeDisplacement[0],-1,1) * (range(relativeDisplacement[0] * strafeP,-1,1));

            motorPowers[0] += -thetaError * strafeRotateCorrection + relativeDisplacement[1] * strafeYCorrection;
            motorPowers[1] += thetaError * strafeRotateCorrection + relativeDisplacement[1] * strafeYCorrection;
            motorPowers[2] += -thetaError * strafeRotateCorrection + relativeDisplacement[1] * strafeYCorrection;
            motorPowers[3] += thetaError * strafeRotateCorrection + relativeDisplacement[1] * strafeYCorrection;

            autonomousRobot.driveBase.setMotorPowers(motorPowers);

            currentPosition = autonomousRobot.odometryTracker.getPosition();

            relativeDisplacement = rotateVec(vectorSub(targetPos, currentPosition), -initPos[2]);

            thetaError = initPos[2] - currentPosition[2];

            telemetry.addData("FL: ", motorPowers[0]);
            telemetry.addData("FR: ", motorPowers[1]);
            telemetry.addData("BL: ", motorPowers[2]);
            telemetry.addData("BR: ", motorPowers[3]);
            telemetry.addData("Relative dx: ", relativeDisplacement[0]);
            telemetry.addData("Relative dy: ", relativeDisplacement[1]);
            telemetry.addData("Theta error", thetaError);
            telemetry.update();

        }

        autonomousRobot.driveBase.setMotorPowers(new double[] {0, 0, 0, 0});

        /*
        telemetry.addData("Done","Doneeee");
        telemetry.addData("Relative dx: ", relativeDisplacement[0]);
        telemetry.addData("Relative dy: ", relativeDisplacement[1]);
        telemetry.addData("dPos x: ", dPos[0]);
        telemetry.addData("dPos y: ", dPos[1]);
        telemetry.addData("Target x",targetPos[0]);
        telemetry.addData("Target y",targetPos[1]);
        telemetry.addData("Init x", initPos[0]);
        telemetry.addData("Init y", initPos[1]);
        telemetry.update();

         */

        telemetry.addData("Final angle", autonomousRobot.odometryTracker.getPosition()[2]);
        telemetry.addData("Relative dx: ", relativeDisplacement[0]);
        telemetry.addData("Relative dy: ", relativeDisplacement[1]);
        telemetry.update();

    }

    protected void skystoneSleep(double millis) {
        double initTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - millis < initTime)
        { if(autonomousRobot.skyStoneFinder.isVisible()) {break;} }
    }

    protected void flashlight(boolean on) { CameraDevice.getInstance().setFlashTorchMode(on); }


}
