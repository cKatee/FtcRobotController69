package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@TeleOp
public class FieldOriented extends LinearOpMode {

    private RobotClass robot;
    private SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass();
        robot.init(hardwareMap);
        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);

        //drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("ready to start","Press play!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            roadrunnerOdometry.updatePoseEstimate();
            robot.robotPose.setPose2dRoadRunner(roadrunnerOdometry.getPoseEstimate());
            robot.FieldRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            telemetry.update();
        }

    }
}
