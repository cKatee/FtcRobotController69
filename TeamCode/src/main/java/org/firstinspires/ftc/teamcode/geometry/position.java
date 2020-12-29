package org.firstinspires.ftc.teamcode.geometry;



public class position {

    protected double x, y, z;
    public org.firstinspires.ftc.teamcode.geometry.angle angle;
    /**
     * constructor for position
     * @param x x position
     * @param y y position
     * @param radians in radians
     */
    public position(double x, double y, double radians) {
        this.x = x;
        this.y = y;
        this.angle = new angle(radians);
    }
    public position(double x, double y, double z, double radians) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.angle = new angle(radians);
    }

    /**
     construction of ftclib position
     */
    public position(Translation2d translation, Rotation2d rotation) {
        this.x = translation.getX();
        this.y = translation.getY();
        this.angle = new angle(rotation.getRadians());
    }

    /**
     * returns distance from this position to pos
     * @param pos
     * @return distance to pos
     */
    public double distanceToPose(position pos) {
        return Math.sqrt(Math.pow(pos.x - this.x, 2) + Math.pow(pos.y - this.y, 2));
    }

    /**
     * arc tangent 2 angle to pose
     * @param pos
     * @return
     */
    public double returnHeadingToPose(position pos) {
        return Math.atan2(pos.y-this.y,pos.x - this.x);
    }

    public double getAngleDegrees() {
        return angle.getDegrees();
    }

    public double getAngleRadians() {
        return angle.getRadians();
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

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    /**
     * set pose based on primative translation and rotation calsses
     * @param translation
     * @param rotation
     */
    public void setPoseFTClib(Translation2d translation, Rotation2d rotation) {
        this.x = translation.getX();
        this.y = translation.getY();
        this.angle = new angle(rotation.getRadians());
    }

    /**
     * set position based on ftclib pose2d
     * @param pose
     */
    public void setPose2d(Pose2d pose) {
        this.x = pose.getTranslation().getX();
        this.y = pose.getTranslation().getY();
        this.angle = new angle(pose.getRotation().getRadians());
    }

    /**
     * sets position based on roadrunner pose2d
     * @param pose
     */
    public void setPose2dRoadRunner(com.acmerobotics.roadrunner.geometry.Pose2d pose) {
        this.x = pose.getX();
        this.y = pose.getY();
        this.angle.setRadians(pose.getHeading());
    }

    /**
     * calculates the midpoint of this point and the other point
     * @param other the other point
     * @return the mid point
     */
    public position midpoint(position other) {
        double x = (this.x + other.x) / 2;
        double y = (this.y + other.y) / 2;
        double angleRad = this.getAngleRadians();
        return new position(x, y , angleRad);

    }

    public void setAngleRad(double rad) {
        this.angle.setRadians(rad);
    }

    public void setAngleDeg(double deg) {
        this.angle.setDegrees(deg);
    }


    public position plus(position other) {
        return new position(other.x + this.x,other.y + this.y,other.getAngleRadians() + this.angle.getRadians());
    }


}
