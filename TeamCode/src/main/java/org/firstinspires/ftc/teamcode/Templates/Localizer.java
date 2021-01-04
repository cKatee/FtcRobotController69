package org.firstinspires.ftc.teamcode.Templates;

import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public abstract class Localizer {

    public DriveTrain drive;
    protected position pose;

    public Localizer(DriveTrain drive, position startPose) {
        this.drive = drive;
        this.pose = startPose;
    }

    /**
     * updates Localization position
     */
    public void updatePose() {

    }

    public void setPose(position pose) {
        this.pose = pose;
    }

    /**
     * @return position object of localization positiion
     */
    public position getPose() {
        return pose;
    }


}
