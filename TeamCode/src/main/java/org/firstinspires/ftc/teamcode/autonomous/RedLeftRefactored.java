package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.controllers.MotionProfiledPoseStablization;
import org.firstinspires.ftc.teamcode.controllers.PoseStablizationController;
import org.firstinspires.ftc.teamcode.controllers.pathFollower;
import org.firstinspires.ftc.teamcode.geometry.path;
import org.firstinspires.ftc.teamcode.geometry.pathPosition;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.insaneControlTheory.KalmanFilter;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.autonomous.auto.auto_states.to_second_wobble_spot;

@Autonomous
public class RedLeftRefactored extends auto {

    private position IMMEDIATE_TARGET_POSITION = new position(0,0,Math.toRadians(0));

    private auto_states AUTO_STATE = auto_states.START;
    private position start_position = new position(0,0,Math.toRadians(0));
    private position high_goal_so_we_miss_starting_stack = new position(60,6,Math.toRadians(180));
    private position high_goal_general_position = new position(64,-9,Math.toRadians(-172));
    private position beyond_high_goal_spot = new position(high_goal_general_position.getX() + 6, high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());

    private position powershot_general_position = new position(63,-34,Math.toRadians(-174));
    private position high_goal_1 = new position(high_goal_general_position.getX(),high_goal_general_position.getY() ,high_goal_general_position.getAngleRadians());
    private position high_goal_2 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
    private position high_goal_3 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
    private position wobble_goal_spot;
    private position to_second_wobble_avoid_stack = new position(40,-30,Math.toRadians(0));
    private position second_wobble_goal = new position(26,-35,Math.toRadians(0));
    private position on_way_to_second_wobble_spot = new position(50,-30,Math.toRadians(0));
    private position to_line_avoid_wobble_goals;
    private position line = new position(70,-12.25,Math.toRadians(180));
    private position shoot_starting_stack_position = high_goal_general_position;

    private position ring_stack = new position(50,-10,Math.toRadians(180));
    private position straight_behind_ring = new position(60,ring_stack.getY(),Math.toRadians(180));
    private position infront_of_ring_stack = new position(high_goal_general_position.getX(),ring_stack.getY(),ring_stack.getAngleRadians());

    private path ringPickupPath = new path();

    private path pathToSecondWG = new path();

    private pathPosition first_ring_of_stack = new pathPosition(ring_stack.getX() - 4,ring_stack.getY(),ring_stack.getAngleRadians(),1.8);
    private pathPosition second_ring_of_stack = new pathPosition(ring_stack.getX() - 9, ring_stack.getY(), ring_stack.getAngleRadians(),1.8);
    private pathPosition third_ring_of_stack = new pathPosition(ring_stack.getX() - 12, ring_stack.getY(), ring_stack.getAngleRadians(),1.8);

    private position robotPositionAtWobbleGoalPlace;


    private double selectedShootingRPM = robot.normflywheelspeed;
    private double selectedShootingTPM = robot.flywheelticksperminute;


    private double arm_position = 0;
    private long timeOfWobbleDelivery1Start = 0;


    private long time_of_shot_1 = 0;
    private long time_of_shot_2 = 0;
    private long time_of_shot_arrival = 0;
    private long time_of_extra_shot = 0;

    private long time_of_shot_1_actuation = 0;
    private long time_of_shot_2_actuation = 0;
    private long time_of_shot_3_actuation = 0;
    /**
     * timersss
     */

    // time in milliseconds allowed for the robot to turn towards the target
    private long time_for_shot_adjust = 400;
    // time in milliseconds for the servo to push the disk
    private long shooter_actuation_time = 350;
    private long timeForWobbleDelivery = 1500;//3 * 1000;
    private long timeOfWobblePlace = 0;
    private long time_between_shots = 200;
    private RingDetector.Height stack;

    private MotionProfiledPoseStablization poseStablizationController = new MotionProfiledPoseStablization(robot);
    private pathFollower pathFollowingController = new pathFollower(poseStablizationController);






    @Override
    public void runOpMode() {
        ringPickupPath.addPosition(ring_stack);
        ringPickupPath.addPosition(first_ring_of_stack);
        ringPickupPath.addPosition(ring_stack);
        ringPickupPath.addPosition(second_ring_of_stack);
        ringPickupPath.addPosition(ring_stack);
        ringPickupPath.addPosition(third_ring_of_stack);

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        roadrunnerOdometry.setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new RingDetector());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        initialize();


        while (!opModeIsActive() && !isStopRequested()) {
            stack = RingDetector.height;
            switch (stack) {
                case ZERO:


                    selectedShootingRPM = robot.powerShotSpeed;
                    selectedShootingTPM = robot.powerShotTicksPerMinute;
                    high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() - 5.5, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 1, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 9, powershot_general_position.getAngleRadians() + Math.toRadians(5));


                    second_wobble_goal = new position(27,-25,Math.toRadians(0));
                    wobble_goal_spot = new position(63,7,Math.toRadians(-165));
                    break;
                case ONE:

                    /*
                    selectedShootingRPM = robot.powerShotSpeed;
                    selectedShootingTPM = robot.powerShotTicksPerMinute;
                    high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() - 3, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 6, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 14, powershot_general_position.getAngleRadians() + Math.toRadians(5));


                     */
                    selectedShootingRPM = robot.powerShotSpeed;
                    selectedShootingTPM = robot.powerShotTicksPerMinute;
                    high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() - 5.5, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 1, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 9, powershot_general_position.getAngleRadians() + Math.toRadians(5));


                    second_wobble_goal = new position(27,-27,Math.toRadians(0));
                    wobble_goal_spot = new position(87.5,-12,Math.toRadians(-170));
                    break;
                case FOUR:



                    selectedShootingRPM = robot.normflywheelspeed;
                    selectedShootingTPM = robot.flywheelticksperminute;
                    high_goal_1 = new position(high_goal_general_position.getX(),high_goal_general_position.getY() ,high_goal_general_position.getAngleRadians());
                    high_goal_2 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
                    high_goal_3 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());



                    wobble_goal_spot = new position(106.5,9,Math.toRadians(-170));
                    second_wobble_goal = new position(28,-23,Math.toRadians(0));
                    break;
                default:

                    /*
                    selectedShootingRPM = robot.powerShotSpeed;
                    selectedShootingTPM = robot.powerShotTicksPerMinute;
                    high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() , powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 7, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 14, powershot_general_position.getAngleRadians() + Math.toRadians(5));


                     */

                    second_wobble_goal = new position(27,-27,Math.toRadians(0));
                    wobble_goal_spot = new position(87.5,-12,Math.toRadians(-170));

                    break;

            }
            telemetry.addData("ring height is: ",stack);
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()) {

            roadrunnerOdometry.updatePoseEstimate();
            Pose2d position_estimate = roadrunnerOdometry.getPoseEstimate();
            robot.robotPose.setPose2dRoadRunner(position_estimate);
            switch (AUTO_STATE) {
                case START:

                    if (poseStablizationController.goToPosition(start_position, 4)) {
                        AUTO_STATE = auto_states.DRIVE_TO_AVOID_STARTING_STACK;
                    }
                    break;
                case DRIVE_TO_AVOID_STARTING_STACK:

                    if (poseStablizationController.goToPosition(high_goal_so_we_miss_starting_stack, 2.5)) {
                        AUTO_STATE = auto_states.DRIVE_TO_POWER_SHOT;
                    }
                    robot.shooter.setVelocity(selectedShootingTPM);

                    break;
                case DRIVE_TO_POWER_SHOT:
                    if (poseStablizationController.goToPosition(high_goal_general_position, 2)) {
                        AUTO_STATE = auto_states.SHOOT_FIRST_POWER_SHOT;
                        robot.drive.STOP();
                        time_of_shot_arrival = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(selectedShootingTPM);
                    break;
                case SHOOT_FIRST_POWER_SHOT:
                    poseStablizationController.goToPosition(high_goal_1, 1.35);
                    if (((robot.shooter.getVelocity() / 28) * 60) > selectedShootingRPM - 80  && Math.abs(robot.yError) < 1.35) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        AUTO_STATE = auto_states.SHOOT_SECOND_POWER_SHOT;
                        time_of_shot_1 = System.currentTimeMillis();

                    }

                    break;
                case SHOOT_SECOND_POWER_SHOT:
                    poseStablizationController.goToPosition(high_goal_2,1.35);
                    if (((robot.shooter.getVelocity() / 28) * 60) > selectedShootingRPM - 80 && (System.currentTimeMillis() - time_of_shot_1) > time_between_shots && Math.abs(robot.yError) < 1.35) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        AUTO_STATE = auto_states.SHOOT_THIRD_POWER_SHOT;
                        time_of_shot_2 = System.currentTimeMillis();
                        sleep(shooter_actuation_time / 2);

                    }
                    break;
                case SHOOT_THIRD_POWER_SHOT:
                    poseStablizationController.goToPosition(high_goal_3, 1.35);
                    if (((robot.shooter.getVelocity() / 28) * 60) > selectedShootingRPM - 80 && (System.currentTimeMillis() - time_of_shot_2) > time_between_shots + 200 && Math.abs(robot.yError) < 1.35) {
                        robot.drive.STOP();
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);

                        AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE;
                        sleep(200);
                    }

                    break;
                case DRIVE_TO_WOBBLE_ZONE:
                    if (poseStablizationController.goToPosition(new position(wobble_goal_spot.getX() + 5,wobble_goal_spot.getY() + 5,wobble_goal_spot.getAngleRadians()), 3)) {
                        AUTO_STATE = auto_states.PLACE_WOBBLE_GOAL;
                        timeOfWobbleDelivery1Start = System.currentTimeMillis();
                        robot.drive.STOP();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_DOWN;
                    break;
                case PLACE_WOBBLE_GOAL:

                    if (System.currentTimeMillis() - timeOfWobbleDelivery1Start > 700) {
                        robot.claw.setPosition(robot.CLAW_OPEN);
                    }
                    if ((timeForWobbleDelivery + timeOfWobbleDelivery1Start) < System.currentTimeMillis()) {
                        AUTO_STATE = auto_states.GO_TO_SECOND_WOBBLE_GOAL_AVOID_STACK;
                        robotPositionAtWobbleGoalPlace = new position(roadrunnerOdometry.getPoseEstimate());
                        pathToSecondWG.addPosition(new pathPosition(robotPositionAtWobbleGoalPlace.getX() - 10, robotPositionAtWobbleGoalPlace.getY(),Math.toRadians(180)));
                        pathToSecondWG.addPosition(to_second_wobble_avoid_stack);
                        pathFollowingController.setTargetPath(pathToSecondWG);
                        // ensure we go back 10x from where we are so we dont make the wobble goal die
                        timeOfWobblePlace = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_DOWN;
                    robot.wrist.setPosition(robot.WRIST_FOR_GRAB);
                    break;
                case GO_TO_SECOND_WOBBLE_GOAL_AVOID_STACK:
                    if (pathFollowingController.followPath()) {
                        AUTO_STATE = auto_states.GO_TO_SECOND_WOBBLE_GOAL;
                    }
                    break;
                case GO_TO_SECOND_WOBBLE_GOAL:
                    if (poseStablizationController.goToPosition(second_wobble_goal,2)) {
                        robot.drive.STOP();
                        AUTO_STATE = auto_states.GRAB_SECOND_WOBBLE_GOAL;
                    }
                    break;

                case GRAB_SECOND_WOBBLE_GOAL:
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    sleep(700);
                    arm_position = robot.LIFT_MID;
                    AUTO_STATE = auto_states.AVOID_RING_STACK_TO_WOBBLE_SPOT_AGAIN;
                    break;


                case AVOID_RING_STACK_TO_WOBBLE_SPOT_AGAIN:

                    if (poseStablizationController.goToPosition(new position(to_second_wobble_avoid_stack.getX() , to_second_wobble_avoid_stack.getY() - 10, to_second_wobble_avoid_stack.getAngleRadians()),3)) {
                        AUTO_STATE = auto_states.to_second_wobble_spot;
                    }
                    break;
                case to_second_wobble_spot:
                    if (poseStablizationController.goToPosition(on_way_to_second_wobble_spot,3)) {
                        AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE_AGAIN;
                    }
                    break;
                case DRIVE_TO_WOBBLE_ZONE_AGAIN:

                    if (poseStablizationController.goToPosition(new position(wobble_goal_spot.getX() - 10,wobble_goal_spot.getY() -10,wobble_goal_spot.getAngleRadians()),3)) {
                        AUTO_STATE = auto_states.PLACE_WOBBLE_GOAL_2;
                        timeOfWobbleDelivery1Start = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_MID;
                    break;

                case PLACE_WOBBLE_GOAL_2:
                    poseStablizationController.goToPosition(new position(wobble_goal_spot.getX() - 10,wobble_goal_spot.getY() -10,wobble_goal_spot.getAngleRadians()),3);

                    if (System.currentTimeMillis() - timeOfWobbleDelivery1Start > 700) {
                        robot.claw.setPosition(robot.CLAW_OPEN);
                    }
                    if ((timeForWobbleDelivery + timeOfWobbleDelivery1Start) < System.currentTimeMillis()) {
                        AUTO_STATE = auto_states.AVOID_PLACED_GOALS;
                        // ensure we go back 10x from where we are so we dont make the wobble goal die
                        to_line_avoid_wobble_goals = new position(robot.robotPose.getX() - 10,robot.robotPose.getY(),Math.toRadians(180));
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_DOWN;
                    robot.wrist.setPosition(robot.WRIST_FOR_PLACING_OUTSIDE);
                    robot.wrist.setPosition(robot.WRIST_FOR_GRAB);
                    break;
                case AVOID_PLACED_GOALS:

                    if (poseStablizationController.goToPosition(to_line_avoid_wobble_goals,4)) {
                        if (stack.equals(RingDetector.Height.ZERO)) {
                            AUTO_STATE = auto_states.PARK_ON_LINE;
                        } else {
                            AUTO_STATE = auto_states.GO_INFRONT_OF_RING;
                            robot.ring_bumper.setPosition(robot.RING_BUMPER_OUT);
                        }
                        timeOfWobblePlace = System.currentTimeMillis();

                    }
                    break;
                case GO_INFRONT_OF_RING:
                    if (poseStablizationController.goToPosition(infront_of_ring_stack,3)) {
                        AUTO_STATE = auto_states.PICK_UP_SECOND_RING;
                        pathFollowingController.setTargetPath(ringPickupPath);
                    }
                    break;
                case PICK_UP_SECOND_RING:

                    if (pathFollowingController.followPath()) {
                        robot.drive.STOP();
                        AUTO_STATE = auto_states.JANK_JERK_THING_POG;
                    }
                    if (timeOfWobblePlace + 1000 < System.currentTimeMillis()) {
                        arm_position = robot.LIFT_IN;
                    } else {
                        arm_position = robot.LIFT_MAX;
                    }
                    robot.intake.setPower(1);

                    break;

                case JANK_JERK_THING_POG:
                    poseStablizationController.updateMovement(beyond_high_goal_spot);
                    if (poseStablizationController.isRobotWithinAllowedTolerance(beyond_high_goal_spot,2)) {
                        AUTO_STATE = auto_states.DRIVE_TO_SHOOTING_POSITION_AGAIN;
                        robot.drive.STOP();
                    }
                    break;

                case DRIVE_TO_SHOOTING_POSITION_AGAIN:
                    robot.shooter.setVelocity(robot.secondSpeedflywheelticksperminute);
                    if (poseStablizationController.goToPosition(shoot_starting_stack_position,2)) {
                        AUTO_STATE = auto_states.SHOOT_RING_AGAIN;
                        time_of_extra_shot = System.currentTimeMillis();
                    }
                    break;




                case SHOOT_RING_AGAIN:


                    poseStablizationController.goToPosition(shoot_starting_stack_position,2);
                    if (robot.robotPose.distanceToPose(shoot_starting_stack_position) < 2.2) {
                        robot.shooter.setVelocity(robot.secondSpeedflywheelticksperminute);
                    }


                    if (((robot.shooter.getVelocity() / 28) * 60) > robot.secondSpeed - 100 && (System.currentTimeMillis() - time_of_extra_shot) > time_between_shots && poseStablizationController.goToPosition(shoot_starting_stack_position,2.5)) {
                        // shoot one or three times
                        double actuation_times = 1;
                        if (stack.equals(RingDetector.Height.FOUR)) {
                            actuation_times = 3;
                        }
                        for (int i = 0; i < actuation_times; i++) {
                            robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                            sleep(shooter_actuation_time * 2);
                            roadrunnerOdometry.updatePoseEstimate();
                            robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                            sleep(shooter_actuation_time / 2);
                            roadrunnerOdometry.updatePoseEstimate();
                        }


                        AUTO_STATE = auto_states.PARK_ON_LINE;
                    }

                    break;

                case PARK_ON_LINE:
                    poseStablizationController.goToPosition(line,2.5);
                    robot.shooter.setVelocity(0);
                    robot.ring_bumper.setPosition(robot.RING_BUMPER_IN);
                    if (timeOfWobblePlace + 2000 < System.currentTimeMillis()) {
                        arm_position = robot.LIFT_IN;
                    } else {
                        arm_position = robot.LIFT_MAX;
                    }
                    robot.wrist.setPosition(robot.WRIST_IN);
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    break;
            }

            robot.driveArmToSetpoint(arm_position);
            telemetry.addData("shooter velo", (robot.shooter.getVelocity() / 28) * 60);
            telemetry.addData("current x", robot.robotPose.getX());
            telemetry.addData("current y", robot.robotPose.getY());
            telemetry.addData("heading error deg", Math.toDegrees(robot.headingError));
            telemetry.addData("current auto state",AUTO_STATE);
            telemetry.addData("stack is: ",stack);
            System.out.println("Current states is" + AUTO_STATE);
            System.out.println("Pose (x,y,theta):  " + robot.robotPose.getX() + ", " + robot.robotPose.getY() + ", " + robot.robotPose.getAngleDegrees());
            telemetry.update();

        }
    }

}
