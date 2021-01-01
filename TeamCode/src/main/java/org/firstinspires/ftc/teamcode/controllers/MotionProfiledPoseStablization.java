package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;

import static org.firstinspires.ftc.teamcode.GlutenCode.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotClass.*;
import static org.firstinspires.ftc.teamcode.RobotClass.headingError;

public class MotionProfiledPoseStablization extends PoseStablizationController {

    // time that the last update occured at
    private double time_of_last_update = 0;

    // has the robot started a motion profiled move to point? this is used

    private boolean hasStarted = false;

    // approx the number of milliseconds one loop takes
    private double loop_time_est = 23;

    // time in milliseconds we take to accelerate
    private double acceleration_time = 700;

    // time of starting the pose motion profiled move

    private double startTime = 0;

    // the amount on each iteration that we increase the power scalar by inorder to ramp acceleration
    private double rate_of_acceleration =  1 / (acceleration_time / loop_time_est);

    // the amount we scale the power by inorder to speedramp; init to rate_of_accel because its a small non-0 number.
    private double powerScaler = rate_of_acceleration;



    public MotionProfiledPoseStablization(RobotClass robot) {
        super(robot);

    }


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


        double d_error_x = (robot.xError - last_error_x) / (currentTime - timeOfLastupdate);
        double d_error_y = (robot.yError - last_error_y) / (currentTime - timeOfLastupdate);
        double d_error_heading = (headingError - last_error_angle) / (currentTime - timeOfLastupdate);

        xPower = ((robot.xError * kp) + (d_error_x * kd)) * powerScaler;
        yPower = ((robot.yError * kp) + (d_error_y * kd)) * powerScaler;
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


    /**
     * is to be run between finishing moving to a position so we can cleanup variables that need to be set in a particular way
     */
    private void cleanUp() {
        hasStarted = false;
        powerScaler = rate_of_acceleration;
    }

    /**
     * calculates new value to ramp up by determined by the loop time
     * @param dt delta time
     * @return the value to increase the power level by
     */
    private double getCurrentRateOfAccel(double dt) {
        return 1 / (acceleration_time / dt);
    }


}
