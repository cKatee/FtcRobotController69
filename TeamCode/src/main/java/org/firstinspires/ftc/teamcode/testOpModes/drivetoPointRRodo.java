package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous
public class drivetoPointRRodo extends LinearOpMode {

    private RobotClass robot;

    private position first = new position(80,1,Math.toRadians(180));
    private position second = new position(41,30,Math.toRadians(180));
    private position third = new position(42,2,Math.toRadians(180));
    private position fourth = new position(1,1,Math.toRadians(180));
    ArrayList<position> path = new ArrayList<>();

    @Override
    public void runOpMode() {


        path.add(first);
        path.add(second);
        path.add(third);
        path.add(fourth);
        //path = robot.adjustedTargetPositionBatch(path, new com.arcrobotics.ftclib.geometry.Pose2d(0,0,new Rotation2d(0)));

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        robot = new RobotClass();
        robot.init(hardwareMap);

        waitForStartReady();
        while (opModeIsActive()) {
            roadrunnerOdometry.updatePoseEstimate();

            Pose2d pose = roadrunnerOdometry.getPoseEstimate();

            robot.robotPose.setPose2dRoadRunner(pose);
            robot.followPath(path);
            telemetry.addData("x",pose.getX());
            telemetry.addData("y",pose.getY());
            telemetry.addData("odom angle",Math.toDegrees(pose.getHeading()) - 180);
            telemetry.addData("IMU",Math.toDegrees(robot.getAngleIMU()));
            telemetry.addData("heading error",robot.headingError);
            telemetry.addData("x Power",robot.xPower);
            telemetry.addData("y Power",robot.yPower);
            telemetry.addData("turn Power",robot.turnPower);
            telemetry.update();

        }


    }

    public void waitForStartReady() {
        telemetry.addData("Ready to Start!","Press play to begin!");
        telemetry.update();
        waitForStart();
    }

}
