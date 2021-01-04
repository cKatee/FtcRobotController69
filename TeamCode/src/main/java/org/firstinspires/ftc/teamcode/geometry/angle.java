package org.firstinspires.ftc.teamcode.geometry;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

public class angle {
    
    protected final double ARC_SECONDS_CONSTANT = 3600;
    protected double radians;
    protected double degrees;
    protected double arcSeconds;

    /**
     * radians constructor 
     * @param radians
     */
    public angle(double radians) {
        this.radians = radians;
        this.degrees = toDegrees(radians);
        this.arcSeconds = degrees * ARC_SECONDS_CONSTANT;
    }


    public double getRadians() {
        return radians;
    }

    public double getDegrees() {
        return degrees;
    }

    public double getArcSeconds() {
        return arcSeconds;
    }

    public void setRadians(double radian) {
        this.radians = radian;
        this.degrees = toDegrees(radian);
        this.arcSeconds = toDegrees(radian) * ARC_SECONDS_CONSTANT;
    }

    public void setDegrees(double degree) {
        this.radians = toRadians(degree);
        this.degrees = degree;
        this.arcSeconds = degree * ARC_SECONDS_CONSTANT;
    }

    public void setArcSeconds(double arcSeconds) {
        this.degrees = arcSeconds / ARC_SECONDS_CONSTANT;
        this.radians = toRadians(this.degrees);
        this.arcSeconds = arcSeconds;
    }



}
