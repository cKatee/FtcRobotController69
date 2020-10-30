package org.firstinspires.ftc.teamcode.geometry;



public class position {

    private double x, y;
    public org.firstinspires.ftc.teamcode.geometry.angle angle;
    /**
     * constructor for position
     * @param x x position
     * @param y y position
     * @param angle in radians
     */
    public position(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = new angle(angle);
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

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
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

    public void setAngleRad(double rad) {
        this.angle.setRadians(rad);
    }

    public void setAngleDeg(double deg) {
        this.angle.setDegrees(deg);
    }



}
