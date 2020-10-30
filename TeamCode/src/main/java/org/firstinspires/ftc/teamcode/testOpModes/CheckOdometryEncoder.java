package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@Autonomous
public class CheckOdometryEncoder extends LinearOpMode {

    private RobotClass robot;


    @Override
    public void runOpMode() {

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        robot = new RobotClass();
        robot.init(hardwareMap);

        waitForStartReady();
        while (opModeIsActive()) {
            roadrunnerOdometry.updatePoseEstimate();

            Pose2d pose = roadrunnerOdometry.getPoseEstimate();
            telemetry.addData("x",pose.getX());
            telemetry.addData("y",pose.getY());
            telemetry.addData("angle",pose.getHeading());
            telemetry.addData("fl",robot.FrontLeft.getCurrentPosition());
            telemetry.addData("fr",robot.FrontRight.getCurrentPosition());
            telemetry.addData("bl",robot.BackLeft.getCurrentPosition());
            telemetry.update();
        }


    }

    public void waitForStartReady() {
        telemetry.addData("Ready to Start!","Press play to begin!");
        telemetry.update();
        waitForStart();
    }

}
