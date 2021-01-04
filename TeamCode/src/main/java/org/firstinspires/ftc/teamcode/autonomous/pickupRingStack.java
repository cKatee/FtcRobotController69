package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.controllers.MotionProfiledPoseStablization;
import org.firstinspires.ftc.teamcode.controllers.PID;
import org.firstinspires.ftc.teamcode.controllers.pathFollower;
import org.firstinspires.ftc.teamcode.geometry.path;
import org.firstinspires.ftc.teamcode.geometry.pathPosition;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class pickupRingStack extends auto {

    private MotionProfiledPoseStablization poseStablizationController = new MotionProfiledPoseStablization(robot);
    private pathFollower pathFollowingController = new pathFollower(poseStablizationController);
    private position ring_stack = new position(0,0,Math.toRadians(180));
    private pathPosition first_ring_of_stack = new pathPosition(ring_stack.getX() - 4,ring_stack.getY(),ring_stack.getAngleRadians(),1.8);
    private pathPosition second_ring_of_stack = new pathPosition(ring_stack.getX() - 9, ring_stack.getY(), ring_stack.getAngleRadians(),1.8);
    private pathPosition third_ring_of_stack = new pathPosition(ring_stack.getX() - 85, ring_stack.getY(), ring_stack.getAngleRadians(),1.8);
    private path ringPickupPath = new path();

    @Override
    public void runOpMode() {

        initialize();

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        roadrunnerOdometry.setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));
        ringPickupPath.addPosition(ring_stack);
        //ringPickupPath.addPosition(first_ring_of_stack);
        //ringPickupPath.addPosition(ring_stack);
        //ringPickupPath.addPosition(second_ring_of_stack);
        //ringPickupPath.addPosition(ring_stack);
        ringPickupPath.addPosition(third_ring_of_stack);
        pathFollowingController.setTargetPath(ringPickupPath);

        waitForStart();

        robot.intake.setPower(1);
        
        while (opModeIsActive()) {

            pathFollowingController.followPath();
            roadrunnerOdometry.updatePoseEstimate();
            robot.robotPose.setPose2dRoadRunner(roadrunnerOdometry.getPoseEstimate());

        }


    }



}
