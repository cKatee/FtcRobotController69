package org.firstinspires.ftc.teamcode.odometry;


import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class ThreeWheelTracking {
    public static double TICKS_PER_REV = 8192;
    public static double OMNI_WHEEL_RADIUS = 1.37795 / 2;
    public static double ODOMETRY_TRACK_WIDTH = 13.189;
    public static double ODOMETRY_MIDDLE_OFFSET = 5.512;
    double current_left_encoder = 0;
    double current_right_encoder = 0;
    double current_aux_encoder = 0;
    double prevLeft = 0;
    double prevRight = 0;
    double prevHorizontal = 0;
    double prevHeading = 0;
    double x = 0;
    double y = 0;
    double radians = 0;


    double angleRadianBias = 0.0;
    /**
     * cunstructor whoop whoop
     * @param start
     */
    public ThreeWheelTracking(Pose2d start) {
        x = start.getTranslation().getX();
        y = start.getTranslation().getY();
        radians = start.getRotation().getRadians();

    }


    /**
     * updates the fucking pose
     * @param leftEncoder
     * @param rightEncoder
     * @param auxEncoder
     */
    public void updatePose(double leftEncoder, double rightEncoder, double auxEncoder, double currentHeading) {
//Get current and delta values
        double curL = -ticksToInches(leftEncoder), curR = -ticksToInches(rightEncoder), curH = -ticksToInches(auxEncoder);
        double dL = curL - prevLeft, dR = curR - prevRight, dH = curH-prevHorizontal;

        //Get and normalize delta in our heading
        double deltaHeading = normalAngle(currentHeading-prevHeading);

        //Get the true distance we travelled using trig
        double robotX = (dL + dR)/2, robotY = dH-ODOMETRY_MIDDLE_OFFSET*deltaHeading;
        double distanceMoved = sqrt(pow(robotX, 2) + pow(robotY,2));
        double angleMoved = atan2(robotY, robotX) - currentHeading;
        double xTravelled = distanceMoved*cos(angleMoved);
        double yTravelled = distanceMoved*sin(angleMoved);

        //Apply the calculations to X and Y
        x+= xTravelled;
        y-= yTravelled;

        prevLeft = curL;
        prevRight = curR;
        prevHorizontal = curH;
        prevHeading = currentHeading;

    }




    public double normalAngle(double target) {
        double PI2 = 2*Math.PI;
        target = (target % PI2 + PI2) % PI2;
        target = (target > Math.PI) ? target-PI2 : target;
        return target;
    }

    /**
     *
     * @return pose 2d current position
     */
    public Pose2d getPose() {
        return new Pose2d(x,y,new Rotation2d(radians));
    }


    public double ticksToInches(double ticks) {
        double inches = OMNI_WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
        return inches;
    }

}
