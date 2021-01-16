package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;

import static org.firstinspires.ftc.teamcode.GlutenCode.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotClass.*;
import static org.firstinspires.ftc.teamcode.RobotClass.headingError;

public class LQRMotionProfiledPoseStabalizationController extends MotionProfiledPoseStablization {

    protected double LQR_K = 7.234;
    protected double LQR_Kr = LQR_K;
    protected double LQR_scaler = 0.01;
    protected double ki = 0.10;
    OneDimensionalLQRController translationLQRx = new OneDimensionalLQRController(LQR_K,LQR_Kr,LQR_scaler,ki);
    OneDimensionalLQRController translationLQRy = new OneDimensionalLQRController(LQR_K,LQR_Kr,LQR_scaler,ki);

    public LQRMotionProfiledPoseStabalizationController(RobotClass robot) {
        super(robot);
    }

    @Override
    public boolean goToPosition(position targetPose, double tolerance) {

        if (hasStarted) {
            double time = System.currentTimeMillis();
            rate_of_acceleration = getCurrentRateOfAccel(time - time_of_last_update);
            time_of_last_update = time;
        } else {

            startTime = System.currentTimeMillis();
            time_of_last_update = startTime;

            hasStarted = true;
        }
        if (powerScaler < 1) {
            powerScaler += rate_of_acceleration;
        } else {
            powerScaler = 1;
        }


        current_target_position = targetPose;

        double currentTime = (double) System.currentTimeMillis() / 1000;




        robot.xError = targetPose.getX() - robotPose.getX();
        robot.yError = targetPose.getY() - robotPose.getY();

        double angle = robot.getAngle();
        double targetAngle = targetPose.getAngleRadians();
        // We are epic so we assume that if the target angle is close to 180 and we are somewhat close to 180 we are at the target angle because we dont fw angle wrap
        headingError = AngleWrap(-robot.normalizeAngleRR(targetAngle - angle));


        double d_error_heading = (headingError - last_error_angle) / (currentTime - timeOfLastupdate);

        xPower = translationLQRx.outputLQR(targetPose.getX(),robotPose.getX()) * powerScaler;
        yPower = translationLQRy.outputLQR(targetPose.getY(),robotPose.getY()) * powerScaler;

        yPower = -yPower;
        turnPower = ((headingError * kpTurn) + (d_error_heading * kdTurn)) * powerScaler;

        robot.FieldRelative(xPower,yPower,turnPower);

        timeOfLastupdate = currentTime;
        last_error_x = robot.xError;
        last_error_y = robot.yError;
        last_error_angle = headingError;

        if (isRobotWithinAllowedToleranceToSetpoint(tolerance)) {
            cleanUp();
        }

        return isRobotWithinAllowedToleranceToSetpoint(tolerance);
    }


}
