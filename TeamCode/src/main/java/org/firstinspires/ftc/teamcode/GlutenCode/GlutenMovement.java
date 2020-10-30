package org.firstinspires.ftc.teamcode.GlutenCode;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;

import static org.firstinspires.ftc.teamcode.GlutenCode.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.RobotClass.robotPose;
import static org.firstinspires.ftc.teamcode.RobotClass.turnPower;
import static org.firstinspires.ftc.teamcode.RobotClass.xPower;
import static org.firstinspires.ftc.teamcode.RobotClass.yPower;

public class GlutenMovement {

    private RobotClass robot;

    public GlutenMovement(RobotClass robot) {
        this.robot = robot;
    }


    public void goToPosition(position targetPose, double point_angle, double movement_speed, double point_speed) {
        double targetX = targetPose.getX();
        double targetY = targetPose.getY();
        //get our distance away from the point
        double distanceToPoint = Math.sqrt(Math.pow(targetX- RobotClass.robotPose.getX(),2) + Math.pow(targetY-RobotClass.robotPose.getY(),2));

        double angleToPoint = Math.atan2(targetY-RobotClass.robotPose.getY(),targetX-RobotClass.robotPose.getX());
        double deltaAngleToPoint = AngleWrap(angleToPoint-(robotPose.getAngleRadians()-Math.toRadians(90)));
        //x and y components required to move toward the next point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relative_abs_x = Math.abs(relative_x_to_point);
        double relative_abs_y = Math.abs(relative_y_to_point);


        //preserve the shape (ratios) of our intended movement direction but scale it by movement_speed
        double movement_x_power = (relative_x_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;
        double movement_y_power = (relative_y_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;

        double movement_turn_power = 0;

        turnPower = movement_turn_power;
        xPower = movement_x_power;
        yPower = movement_y_power;
        double tl_power_raw = yPower-turnPower+xPower*1.5;
        double bl_power_raw = yPower-turnPower- xPower*1.5;
        double br_power_raw = -yPower-turnPower-xPower*1.5;
        double tr_power_raw = -yPower-turnPower+xPower*1.5;

        robot.drive.setMotorPowers(tl_power_raw,tr_power_raw,bl_power_raw,br_power_raw);

    }
}
