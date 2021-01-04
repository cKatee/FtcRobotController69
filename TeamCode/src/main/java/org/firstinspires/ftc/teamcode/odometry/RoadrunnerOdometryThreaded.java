package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

public class RoadrunnerOdometryThreaded implements Runnable {


    protected SampleMecanumDrive rrMecanumDrive;
    protected RobotClass robot;
    protected position estimatedPosition;
    protected boolean stopNotThreadYet = true;
    public RoadrunnerOdometryThreaded(SampleMecanumDrive rrMecanumDrive, RobotClass robot) {
        this.rrMecanumDrive = rrMecanumDrive;
        this.robot = robot;
    }

    @Override
    public void run() {
        while (stopNotThreadYet) {
            rrMecanumDrive.updatePoseEstimate();
            estimatedPosition = new position(rrMecanumDrive.getPoseEstimate());
            robot.robotPose.setPosition(estimatedPosition);
        }
    }

    public void stop() {
        stopNotThreadYet = false;
    }
}
