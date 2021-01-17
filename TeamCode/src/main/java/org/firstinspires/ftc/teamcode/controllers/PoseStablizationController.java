package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;

import static org.firstinspires.ftc.teamcode.GlutenCode.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotClass.*;

public class PoseStablizationController {
    protected double kp = 17.1 * 0.01; // TODO: run printPosition with a 14v battery and get the fastest case transfer function
    double kd = 0.12627 * 0.06;
    protected double ki = 0.11;
    double kpTurn = 2.8;
    double kdTurn = 0;
    protected RobotClass robot;
    protected double last_error_x = 0;
    protected double last_error_y = 0;
    protected double last_error_angle = 0;
    protected double timeOfLastupdate = 0;
    public position current_target_position = new position(0,0,0);

    public PoseStablizationController(RobotClass robot) {
        this.robot = robot;
    }


    /**
     * drives the mecanum robot to 0 steady state error on all three axis
     * @param targetPose
     */
    public void updateMovement(position targetPose) {


        current_target_position = targetPose;

        double currentTime = (double) System.currentTimeMillis() / 1000;


        robot.xError = targetPose.getX() - robotPose.getX();
        robot.yError = targetPose.getY() - robotPose.getY();

        double angle = robot.getAngle();
        double targetAngle = targetPose.getAngleRadians();
        // We are epic so we assume that if the target angle is close to 180 and we are somewhat close to 180 we are at the target angle because we dont fw angle wrap
        headingError = AngleWrap(-robot.normalizeAngleRR(targetAngle - angle));


        double d_error_x = (robot.xError - last_error_x) / (currentTime - timeOfLastupdate);
        double d_error_y = (robot.yError - last_error_y) / (currentTime - timeOfLastupdate);
        double d_error_heading = (headingError - last_error_angle) / (currentTime - timeOfLastupdate);

        xPower = (robot.xError * kp) + (d_error_x * kd);
        yPower = (robot.yError * kp) + (d_error_y * kd);
        yPower = -yPower;
        turnPower = (headingError * kpTurn) + (d_error_heading * kdTurn);

        robot.FieldRelative(xPower,yPower,turnPower);

        timeOfLastupdate = currentTime;
        last_error_x = robot.xError;
        last_error_y = robot.yError;
        last_error_angle = headingError;
    }


    /**
     * moves the robot towards the setpoint until it is within the given tolerance
     * @param targetPose
     * @param tolerance
     * @return true if the robot is within the tolerance
     */
    public boolean updateMovementToTolerance(position targetPose, double tolerance) {
        if (isRobotWithinAllowedTolerance(targetPose,tolerance)) {
            robot.drive.STOP();
            return false;
        } else {
            updateMovement(targetPose);
            return true;
        }

    }

    /**
     * checks if the robot is within a given distance to a given point
     * @param targetPose
     * @param tolerance
     * @return true if within the allowed tolerance
     */
    public boolean isRobotWithinAllowedTolerance(position targetPose, double tolerance) {
        return robotPose.distanceToPose(targetPose) < tolerance;
    }

    /**
     * check if the robot is within tolerance to the cached position
     * @param tolerance
     * @return true if the robot is within the allowed tolerance
     */
    public boolean isRobotWithinAllowedToleranceToSetpoint(double tolerance) {
        return isRobotWithinAllowedTolerance(current_target_position,tolerance);
    }

}
