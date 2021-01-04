package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.controllers.LQRMotionProfiledPoseStabalizationController;
import org.firstinspires.ftc.teamcode.controllers.MotionProfiledPoseStablization;
import org.firstinspires.ftc.teamcode.controllers.PoseStablizationController;
import org.firstinspires.ftc.teamcode.controllers.pathFollower;
import org.firstinspires.ftc.teamcode.geometry.path;
import org.firstinspires.ftc.teamcode.geometry.pathPosition;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@Autonomous
public class pidDriveTuner extends auto {
    private int pathNum = 1;
    private pathPosition path1target1 = new pathPosition(90,0,Math.toRadians(180),1.2);
    private pathPosition path1target2 = new pathPosition(45,20,Math.toRadians(90),1.2);
    private pathPosition path1target3 = new pathPosition(0,0,Math.toRadians(0),1.2);

    private pathPosition path2target1 = new pathPosition(45,20,Math.toRadians(180),1.2);
    private pathPosition path2target2 = new pathPosition(90,0,Math.toRadians(45),1.2);
    private pathPosition path2Target3 = new pathPosition(0,0,Math.toRadians(0),1.2);
    private path targetPath1 = new path();
    private path targetPath2 = new path();



    private ElapsedTime timer = new ElapsedTime();
    private MotionProfiledPoseStablization poseStablizationController = new LQRMotionProfiledPoseStabalizationController(robot);
    private pathFollower pathFollower = new pathFollower(poseStablizationController);


    public void runOpMode() {

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        targetPath1.addPosition(path1target1);
        targetPath1.addPosition(path1target2);
        targetPath1.addPosition(path1target3);

        targetPath2.addPosition(path2target1);
        targetPath2.addPosition(path2target2);
        targetPath2.addPosition(path2Target3);
        pathFollower.setTargetPath(targetPath1);

        initialize();
        waitForStart();
        while (opModeIsActive()) {
            timer.reset();
            roadrunnerOdometry.updatePoseEstimate();
            Pose2d position_estimate = roadrunnerOdometry.getPoseEstimate();
            robot.robotPose.setPose2dRoadRunner(position_estimate);
            boolean followPath = false;

            switch (pathNum) {
                case 1:
                    followPath = pathFollower.followPath();
                    if (followPath) {
                        pathNum += 1;
                        pathFollower.setTargetPath(targetPath2);
                    }
                    break;
                case 2:
                    followPath = pathFollower.followPath();
                    break;
            }
            telemetry.addData("is path complete?",followPath);

            telemetry.addData("x: ",position_estimate.getX());
            telemetry.addData("y: ",position_estimate.getY());
            telemetry.addData("heading error",Math.toDegrees(robot.headingError));
            System.out.println("Loop time is: " + timer.milliseconds());

            telemetry.update();
        }
    }


}
