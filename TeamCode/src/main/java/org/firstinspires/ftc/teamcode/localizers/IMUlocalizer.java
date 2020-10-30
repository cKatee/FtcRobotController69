package org.firstinspires.ftc.teamcode.localizers;

import org.firstinspires.ftc.teamcode.Templates.Localizer;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class IMUlocalizer extends Localizer {

    public IMUlocalizer(DriveTrain drive, position startPose) {
        super(drive, startPose);
    }
}
