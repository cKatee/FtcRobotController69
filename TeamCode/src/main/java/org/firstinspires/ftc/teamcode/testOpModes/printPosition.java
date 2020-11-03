package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@Autonomous
public class printPosition extends LinearOpMode {

    private RobotClass robot;

    @Override
    public void runOpMode() {

        //path = robot.adjustedTargetPositionBatch(path, new com.arcrobotics.ftclib.geometry.Pose2d(0,0,new Rotation2d(0)));

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        robot = new RobotClass();
        robot.init(hardwareMap);

        waitForStartReady();
        double loopTime = (double)System.currentTimeMillis() / 1000;
        double startTime = (double)System.currentTimeMillis() / 1000;
        while (opModeIsActive()) {

            roadrunnerOdometry.updatePoseEstimate();

            Pose2d pose = roadrunnerOdometry.getPoseEstimate();
            Acceleration accel = robot.imu.getLinearAcceleration();
            robot.robotPose.setPose2dRoadRunner(pose);
            robot.FieldRelative(1,0,0);
            telemetry.addData("x",pose.getX());
            telemetry.addData("y",pose.getY());
            telemetry.addData("odom angle",Math.toDegrees(pose.getHeading()) - 180);
            telemetry.addData("IMU",Math.toDegrees(robot.getAngleIMU()));
            telemetry.addData("heading error",robot.headingError);
            telemetry.addData("x Power",robot.xPower);
            telemetry.addData("y Power",robot.yPower);
            telemetry.addData("turn Power",robot.turnPower);
            telemetry.update();
            double currentTime = ((double)System.currentTimeMillis() / 1000) - loopTime;
            System.out.println("" + pose.getX() + ", " +pose.getY() + ", " + currentTime + ", " + accel.xAccel + ", " + accel.yAccel + ", " + (System.currentTimeMillis() - startTime));

            loopTime = (double)System.currentTimeMillis() / 1000;

        }


    }

    public void waitForStartReady() {
        telemetry.addData("Ready to Start!","Press play to begin!");
        telemetry.update();
        waitForStart();
    }

}
