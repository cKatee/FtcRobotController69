package org.firstinspires.ftc.teamcode.geometry;

public class pathPosition extends position {

    // steady state error tolerance
    protected double steady_state_tolerance = 3;

    public pathPosition(double x, double y, double angle) {
        super(x, y, angle);
    }

    public pathPosition(double x, double y, double angle, double steady_state_tolerance) {
        super(x,y,angle);
        this.steady_state_tolerance = steady_state_tolerance;
    }

    public pathPosition(position pos) {
        super(pos.x,pos.y,pos.angle.getRadians());

    }


    public double getSteady_state_tolerance() {
        return steady_state_tolerance;
    }

}
