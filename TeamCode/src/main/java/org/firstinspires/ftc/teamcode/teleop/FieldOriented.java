package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.odometry.ThreeWheelTracking;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@TeleOp
public class FieldOriented extends LinearOpMode {

    private RobotClass robot;
    private SampleMecanumDrive drive;
    private ThreeWheelTracking odometry;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass();
        robot.init(hardwareMap);
        //drive = new SampleMecanumDrive(hardwareMap);
        odometry = new ThreeWheelTracking(new position(0,0,0),robot);
        telemetry.addData("ready to start","Press play!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odometry.updatePose();
            robot.FieldRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            telemetry.addData("x",robot.robotPose.getX());
            telemetry.addData("y",robot.robotPose.getY());
            telemetry.addData("theta",robot.robotPose.getAngleDegrees());
            telemetry.update();
        }

    }
}
