package org.firstinspires.ftc.teamcode.odometry;



import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.geometry.position;


public class ThreeWheelTracking {
    public position robotPosition;
    public static double TICKS_PER_REV = 8192;
    public static double OMNI_WHEEL_RADIUS = 1.37795 / 2;
    public static double ODOMETRY_TRACK_WIDTH = 13.189;
    public static double ODOMETRY_MIDDLE_OFFSET = 5.512;
    public static final double EPSILON = 1e-6;

    double prevLeft = 0;
    double prevRight = 0;
    double prevHorizontal = 0;
    double prevHeading = 0;
    double x = 0;
    double y = 0;
    double radians = 0;
    private RobotClass robot;

    double angleRadianBias = 0.0;
    /**
     * cunstructor whoop whoop
     * @param start
     */
    public ThreeWheelTracking(position start, RobotClass robot) {
        robotPosition = start;
        this.robot = robot;
        robot.robotPose = robotPosition;
    }


    /**
     * updates the fucking pose
     */
    public void updatePose() {
        double leftEncoder = -robot.drive.FrontLeft.getCurrentPosition();
        double rightEncoder = robot.drive.FrontRight.getCurrentPosition();
        double auxEncoder = -robot.drive.BackLeft.getCurrentPosition();


        double deltaL = ticksToInches(leftEncoder - prevLeft);
        double deltaR = ticksToInches(rightEncoder - prevRight);
        double deltaH = ticksToInches(auxEncoder - prevHorizontal);

        double deltaAngle = (deltaL - deltaR) / ODOMETRY_TRACK_WIDTH;
        double deltaX = (deltaL + deltaR) / 2;
        double deltaY = deltaH - ODOMETRY_MIDDLE_OFFSET * deltaAngle;


        double estimateArcAngle = robot.robotPose.getAngleRadians() + deltaAngle / 2;

        double fieldOrientedX = deltaX * Math.cos(estimateArcAngle)  - deltaY * Math.sin(estimateArcAngle);
        double fieldOrientedY = deltaX * Math.sin(estimateArcAngle) + deltaY * Math.cos(estimateArcAngle);

        position deltaPose = new position(fieldOrientedX, fieldOrientedY, deltaAngle);

        robot.robotPose = robot.robotPose.plus(deltaPose);

        prevLeft = leftEncoder;
        prevRight = rightEncoder;
        prevHeading = auxEncoder;

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

    public boolean approxEquals(double v1, double v2) {
        if (Double.isInfinite(v1)) {
            return v1 == v2;
        } else {
            return Math.abs(v1 - v2) < EPSILON;
        }
    }


}
