package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RedLeft extends auto {
    private auto_states AUTO_STATE = auto_states.START;
    private position start_position = new position(0,0,Math.toRadians(0));
    private position high_goal_so_we_miss_starting_stack = new position(60,6,Math.toRadians(180));
    private position high_goal_general_position = new position(57,-8,Math.toRadians(-173));
    private position high_goal_1 = new position(high_goal_general_position.getX(),high_goal_general_position.getY() ,high_goal_general_position.getAngleRadians());
    private position high_goal_2 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
    private position high_goal_3 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
    private position wobble_goal_spot;
    private position to_second_wobble_avoid_stack = new position(50,-35,Math.toRadians(0));
    private position second_wobble_goal = new position(26,-23,Math.toRadians(0));
    private position on_way_to_second_wobble_spot = new position(50,-30,Math.toRadians(0));
    private position to_line_avoid_wobble_goals;
    private position line = new position(70,-25,Math.toRadians(180));
    private position power_shot = new position(high_goal_general_position.getX(), high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());

    private position ring_stack = new position(30,-12,Math.toRadians(180));
    private double arm_position = 0;
    private long timeOfWobbleDelivery1Start = 0;


    private long time_of_shot_1 = 0;
    private long time_of_shot_2 = 0;
    private long time_of_shot_arrival = 0;
    private long time_of_extra_shot = 0;
    /**
     * timersss
     */

    // time in milliseconds allowed for the robot to turn towards the target
    private long time_for_shot_adjust = 400;
    // time in milliseconds for the servo to push the disk
    private long shooter_actuation_time = 700;
    private long timeForWobbleDelivery = 1500;//3 * 1000;
    private long timeOfWobblePlace = 0;
    private long time_between_shots = 700;
    private RingDetector.Height stack;

    @Override
    public void runOpMode() {
        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);

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
                    wobble_goal_spot = new position(62,6,Math.toRadians(180));
                    break;
                case ONE:
                    wobble_goal_spot = new position(81,-18,Math.toRadians(-170));
                    break;
                case FOUR:
                    wobble_goal_spot = new position(104,6,Math.toRadians(-170));
                    second_wobble_goal = new position(26,-26.9,Math.toRadians(0));
                    break;
                default:
                    // default to zone B
                    wobble_goal_spot = new position(81,-18,Math.toRadians(-170));
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
                    robot.goodDriveToPoint(start_position);
                    if (robot.robotPose.distanceToPose(start_position) < 4) {
                        AUTO_STATE = auto_states.DRIVE_TO_AVOID_STARTING_STACK;
                    }
                    break;
                case DRIVE_TO_AVOID_STARTING_STACK:
                    robot.goodDriveToPoint(high_goal_so_we_miss_starting_stack);
                    if (robot.robotPose.distanceToPose(high_goal_so_we_miss_starting_stack) < 2.5) {
                        AUTO_STATE = auto_states.DRIVE_TO_POWER_SHOT;
                    }
                    break;
                case DRIVE_TO_POWER_SHOT:
                    robot.goodDriveToPoint(high_goal_general_position);

                    if (robot.robotPose.distanceToPose(high_goal_general_position) < 2) {
                        AUTO_STATE = auto_states.SHOOT_FIRST_POWER_SHOT;
                        robot.drive.STOP();
                        time_of_shot_arrival = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    break;
                case SHOOT_FIRST_POWER_SHOT:
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    robot.goodDriveToPoint(high_goal_1);


                    if (((robot.shooter.getVelocity() / 28) * 60) > 4400 && (System.currentTimeMillis() - (time_of_shot_arrival + 1000)) > time_between_shots) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2);

                        AUTO_STATE = auto_states.SHOOT_SECOND_POWER_SHOT;
                        time_of_shot_1 = System.currentTimeMillis();
                    }
                    break;
                case SHOOT_SECOND_POWER_SHOT:
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    robot.goodDriveToPoint(high_goal_2);

                    if (((robot.shooter.getVelocity() / 28) * 60) > 4400 && (System.currentTimeMillis() - time_of_shot_1) > time_between_shots) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2 );
                        AUTO_STATE = auto_states.SHOOT_THIRD_POWER_SHOT;
                        time_of_shot_2 = System.currentTimeMillis();
                    }
                    break;
                case SHOOT_THIRD_POWER_SHOT:
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    robot.goodDriveToPoint(high_goal_3);

                    if (((robot.shooter.getVelocity() / 28) * 60) > 4400 && (System.currentTimeMillis() - time_of_shot_2) > time_between_shots) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2);

                        AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE;
                    }

                    break;
                case DRIVE_TO_WOBBLE_ZONE:
                    if (robot.robotPose.distanceToPose(new position(wobble_goal_spot.getX() + 5,wobble_goal_spot.getY() + 8,wobble_goal_spot.getAngleRadians())) < 4) {
                        AUTO_STATE = auto_states.PLACE_WOBBLE_GOAL;
                        timeOfWobbleDelivery1Start = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_MID;
                    robot.goodDriveToPoint(new position(wobble_goal_spot.getX() + 5,wobble_goal_spot.getY() + 8,wobble_goal_spot.getAngleRadians()));
                    break;
                case PLACE_WOBBLE_GOAL:
                    if (System.currentTimeMillis() - timeOfWobbleDelivery1Start > 700) {
                        robot.claw.setPosition(robot.CLAW_OPEN);
                    }
                    if ((timeForWobbleDelivery + timeOfWobbleDelivery1Start) < System.currentTimeMillis()) {
                        AUTO_STATE = auto_states.GO_TO_SECOND_WOBBLE_GOAL_AVOID_STACK;
                        // ensure we go back 10x from where we are so we dont make the wobble goal die
                        timeOfWobblePlace = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_DOWN;
                    robot.wrist.setPosition(robot.WRIST_FOR_PLACING_OUTSIDE);
                    robot.wrist.setPosition(robot.WRIST_FOR_GRAB);
                    robot.goodDriveToPoint(wobble_goal_spot);
                    break;
                case GO_TO_SECOND_WOBBLE_GOAL_AVOID_STACK:
                    robot.goodDriveToPoint(to_second_wobble_avoid_stack);
                    if (robot.robotPose.distanceToPose(to_second_wobble_avoid_stack) < 4) {
                        AUTO_STATE = auto_states.GO_TO_SECOND_WOBBLE_GOAL;
                    }
                    break;
                case GO_TO_SECOND_WOBBLE_GOAL:
                    robot.goodDriveToPointWithMaxSpeed(second_wobble_goal,0.65);
                    if (robot.robotPose.distanceToPose(second_wobble_goal) < 1.5) {
                        AUTO_STATE = auto_states.GRAB_SECOND_WOBBLE_GOAL;
                    }
                    break;

                case GRAB_SECOND_WOBBLE_GOAL:
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    sleep(700);
                    arm_position = robot.LIFT_MID;
                    AUTO_STATE = auto_states.to_second_wobble_spot;
                    break;

                case to_second_wobble_spot:
                    robot.goodDriveToPoint(on_way_to_second_wobble_spot);
                    if (robot.robotPose.distanceToPose(on_way_to_second_wobble_spot) < 3) {
                        AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE_AGAIN;
                    }
                    break;
                case DRIVE_TO_WOBBLE_ZONE_AGAIN:
                    if (robot.robotPose.distanceToPose(new position(wobble_goal_spot.getX() - 10,wobble_goal_spot.getY(),wobble_goal_spot.getAngleRadians())) < 4) {
                        AUTO_STATE = auto_states.PLACE_WOBBLE_GOAL_2;
                        timeOfWobbleDelivery1Start = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_MID;
                    robot.goodDriveToPoint(new position(wobble_goal_spot.getX() - 10,wobble_goal_spot.getY(),wobble_goal_spot.getAngleRadians()));
                    break;

                case PLACE_WOBBLE_GOAL_2:
                    if (System.currentTimeMillis() - timeOfWobbleDelivery1Start > 700) {
                        robot.claw.setPosition(robot.CLAW_OPEN);
                    }
                    if ((timeForWobbleDelivery + timeOfWobbleDelivery1Start) < System.currentTimeMillis()) {
                        AUTO_STATE = auto_states.AVOID_PLACED_GOALS;
                        // ensure we go back 10x from where we are so we dont make the wobble goal die
                        to_line_avoid_wobble_goals = new position(robot.robotPose.getX() - 20,robot.robotPose.getY(),robot.robotPose.getAngleRadians());
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_DOWN;
                    robot.wrist.setPosition(robot.WRIST_FOR_PLACING_OUTSIDE);
                    robot.wrist.setPosition(robot.WRIST_FOR_GRAB);
                    robot.goodDriveToPoint(wobble_goal_spot);
                    break;
                case AVOID_PLACED_GOALS:

                    robot.goodDriveToPoint(to_line_avoid_wobble_goals);
                    if (robot.robotPose.distanceToPose(to_line_avoid_wobble_goals) < 4) {
                        if (stack.equals(RingDetector.Height.ONE)) {
                            AUTO_STATE = auto_states.PICK_UP_SECOND_RING;
                        } else {
                            AUTO_STATE = auto_states.PARK_ON_LINE;
                        }
                        timeOfWobblePlace = System.currentTimeMillis();

                    }
                    break;
                case PICK_UP_SECOND_RING:
                    if (timeOfWobblePlace + 1000 < System.currentTimeMillis()) {
                        arm_position = robot.LIFT_IN;
                    } else {
                        arm_position = robot.LIFT_MAX;
                    }
                    robot.goodDriveToPoint(ring_stack);
                    robot.intake.setPower(1);
                    if (robot.robotPose.distanceToPose(ring_stack) < 3) {
                        AUTO_STATE = auto_states.DRIVE_TO_SHOOTING_POSITION_AGAIN;
                    }
                    break;
                case DRIVE_TO_SHOOTING_POSITION_AGAIN:
                    robot.shooter.setVelocity(robot.secondSpeedflywheelticksperminute);
                    robot.goodDriveToPoint(power_shot);
                    if (robot.robotPose.distanceToPose(power_shot) < 2.5) {
                        AUTO_STATE = auto_states.SHOOT_RING_AGAIN;
                        time_of_extra_shot = System.currentTimeMillis();
                    }
                case SHOOT_RING_AGAIN:
                    robot.goodDriveToPoint(power_shot);

                    if (((robot.shooter.getVelocity() / 28) * 60) > robot.secondSpeed - 100 && (System.currentTimeMillis() - time_of_extra_shot) > time_between_shots && robot.robotPose.distanceToPose(power_shot) < 2) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2);

                        AUTO_STATE = auto_states.PARK_ON_LINE;
                    }

                    break;


                case PARK_ON_LINE:
                    robot.shooter.setVelocity(0);
                    if (timeOfWobblePlace + 2000 < System.currentTimeMillis()) {
                        arm_position = robot.LIFT_IN;
                    } else {
                        arm_position = robot.LIFT_MAX;
                    }
                    robot.wrist.setPosition(robot.WRIST_IN);
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    robot.goodDriveToPoint(line);
                    break;
            }
            robot.driveArmToSetpoint(arm_position);
            telemetry.addData("shooter velo", (robot.shooter.getVelocity() / 28) * 60);
            telemetry.addData("current x", robot.robotPose.getX());
            telemetry.addData("current y", robot.robotPose.getY());
            telemetry.addData("angle", robot.getAngleIMU());
            telemetry.addData("heading error deg", Math.toDegrees(robot.headingError));
            telemetry.addData("current auto state",AUTO_STATE);
            telemetry.addData("stack is: ",stack);
            telemetry.update();

        }
    }

}
