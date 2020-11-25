package org.firstinspires.ftc.teamcode.localizers;

public class IMUlocalizer {


    double x = 0;
    double y = 0;
    double z = 0;

    double xvelo = 0;
    double yvelo = 0;
    double zvelo = 0;

    double angleRad = 0;

    double last_xaccel_measurement = 0;
    double last_yaccel_measurement = 0;
    double last_zaccel_measurement = 0;
    double last_xvelo_measurement = 0;
    double last_yvelo_measurement = 0;
    double last_zvelo_measurement = 0;

    double time_of_last_measurement = 0;

    public IMUlocalizer(double startX, double startY, double startZ, double startRadians) {
        this.x = startX;
        this.y = startY;
        this.z = startZ;
        this.angleRad = startRadians;

    }




    public void updatePoseEstimate(double xaccel_robot, double yaccel_robot, double radians) {


        double cosA = Math.cos(radians);
        double sinA = Math.sin(radians);
        double xaccel  = xaccel_robot * cosA - yaccel_robot * sinA;
        double yaccel = xaccel_robot * sinA + yaccel_robot * cosA;


        double time_of_current_measurement = (double) System.currentTimeMillis() / 1000;
        double time_delta = time_of_current_measurement - time_of_last_measurement;

        xvelo += ((xaccel + last_xaccel_measurement) / 2 ) * time_delta;
        yvelo += ((yaccel + last_yaccel_measurement) / 2 ) * time_delta;



        x += ((xvelo + last_xvelo_measurement) / 2 ) * time_delta;
        y += ((yvelo + last_yvelo_measurement) / 2 ) * time_delta;

        last_xaccel_measurement = xaccel;
        last_yaccel_measurement = yaccel;

        last_xvelo_measurement = xvelo;
        last_yvelo_measurement = yvelo;
        last_zvelo_measurement = zvelo;

        time_of_last_measurement = time_of_current_measurement;

    }


    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getAngleRad() {
        return angleRad;
    }
}
