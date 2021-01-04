package org.firstinspires.ftc.teamcode.localizers;

import org.firstinspires.ftc.teamcode.Templates.Localizer;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class tankDriveLocalizer extends Localizer {


    protected double fr, fl, bl, br, theta;

    protected double Lastfr, Lastfl, Lastbl, Lastbr, Lasttheta;


    public tankDriveLocalizer(DriveTrain drive, position startPose) {
        super(drive, startPose);

        Lastfr = drive.FrontRight.getCurrentPosition();
        Lastfl = drive.FrontLeft.getCurrentPosition();
        Lastbl = drive.BackLeft.getCurrentPosition();
        Lastbr = drive.BackRight.getCurrentPosition();
        Lasttheta = 0;
    }

    @Override
    public void updatePose() {

        fr = drive.FrontRight.getCurrentPosition() - Lastfr;
        fl = drive.FrontLeft.getCurrentPosition() - Lastfl;
        br = drive.BackRight.getCurrentPosition() - Lastbr;
        bl = drive.BackLeft.getCurrentPosition() - Lastbl;

        double deltaL = (bl + fl) / 2;
        double deltaR = (br + fr) / 2;

        double RF = (deltaL + deltaR) / 2;
        double Rtheta = (Math.pow(deltaL,2) - deltaR) / drive.TRACK_WIDTH;

        theta = theta + Rtheta;

        double xp = getPose().getX() + (RF * Math.cos(theta));
        double yp = getPose().getY() + (RF * Math.sin(theta));

        setPose(new position(xp, yp, theta));

    }

}
