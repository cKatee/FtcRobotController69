package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controllers.MotionProfiledPoseStablization;
import org.firstinspires.ftc.teamcode.controllers.PoseStablizationController;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@Autonomous
public class pidDriveTuner extends auto {

    private position target = new position(90,0,Math.toRadians(180));
    private MotionProfiledPoseStablization poseStablizationController = new MotionProfiledPoseStablization(robot);
    public void runOpMode() {

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);

        initialize();
        waitForStart();
        while (opModeIsActive()) {
            roadrunnerOdometry.updatePoseEstimate();
            Pose2d position_estimate = roadrunnerOdometry.getPoseEstimate();
            robot.robotPose.setPose2dRoadRunner(position_estimate);

            poseStablizationController.goToPosition(target,0.6);

            telemetry.addData("x: ",position_estimate.getX());
            telemetry.addData("y: ",position_estimate.getY());
            telemetry.addData("heading error",Math.toDegrees(robot.headingError));
            telemetry.addData("target x: ",target.getX());
            telemetry.addData("target y: ",target.getY());
            telemetry.addData("target heading",target.getAngleDegrees());

            telemetry.update();
        }
    }


}
