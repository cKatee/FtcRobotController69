package org.firstinspires.ftc.teamcode.controllers;

public class PID {

    private double kp;
    private double ki;
    private double kd;

    private double p_error = 0;
    private double i_error = 0;
    private double d_error = 0;
    private double last_error = 0;
    private long last_time = 0;
    private long current_time = 0;
    private long time_duration = 0;
    private double scalar = 1;

    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }


    public void setScalar(double scalar) {
        this.scalar = scalar;
    }

    public double update(double state, double setpoint) {
        current_time = System.currentTimeMillis() / 1000;
        double output;
        double error = setpoint - state;
        time_duration = current_time - last_time;
        p_error = error;
        d_error = (error - last_error) / time_duration;
        // we assume the integral is in the shape of a trapezoid and take that area instead
        double trapezoidArea = error * time_duration;
        i_error = i_error + (trapezoidArea);

        output = (p_error * kp) + (d_error * kd) + (i_error * ki);
        last_error = error;
        last_time = current_time;

        return output;
    }
}
