package org.firstinspires.ftc.teamcode.GlutenCode;

public class utils {
    /**
     * Makes sure an angle is in the range of -180 to 180
     * @param angle
     * @return
     */
    public static double AngleWrap(double angle){
        while (angle<=-Math.PI){
            angle += 2.0*Math.PI;
        }
        while (angle>=Math.PI){
            angle -= 2.0*Math.PI;
        }
        return angle;
    }

    public static double AngleWRapDeg(double degrees) {
        while (degrees < -180) {
            degrees += 2.0 * 180;
        }
        while (degrees > 180) {
            degrees -= 2.0 * 180;
        }
        return degrees;
    }
}
