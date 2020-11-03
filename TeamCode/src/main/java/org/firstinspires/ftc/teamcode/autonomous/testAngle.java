package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.GlutenCode.utils.AngleWrap;

@Autonomous
public class testAngle extends auto {

    @Override
    public void runOpMode() {
        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);


        initialize();
        waitForStart();
        while (opModeIsActive()) {
            roadrunnerOdometry.updatePoseEstimate();
            Pose2d position_estimate = roadrunnerOdometry.getPoseEstimate();
            robot.robotPose.setPose2dRoadRunner(position_estimate);
            telemetry.addData("angle error: ",Math.toDegrees(AngleWrap(robot.getAngleIMU() - Math.toRadians(172))));
            telemetry.addData("angle: ",Math.toDegrees(robot.getAngleIMU()));
            telemetry.addData("rr angle",Math.toDegrees(robot.getAngleProper()));
            telemetry.update();
        }
    }
}
