package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.RingDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.vision.RingDetector.Height.FOUR;
import static org.firstinspires.ftc.teamcode.vision.RingDetector.Height.ZERO;

@Autonomous
public class BlueLeft7Ring extends auto {
    private auto_states AUTO_STATE = auto_states.START;
    private position start_position = new position(0,0,Math.toRadians(0));
    private position high_goal_so_we_miss_starting_stack = new position(60,6,Math.toRadians(180));
    private position high_goal_general_position = new position(58,-3,Math.toRadians(-174));

    private position powershot_general_position = new position(65,-37,Math.toRadians(-174));

    // stuff for making the ring jerk back into the robot
    private position beyond_high_goal_spot = new position(high_goal_general_position.getX() + 6, high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
    ArrayList<position> high_goal_after_pickup_intentional_jerk = new ArrayList<>();

    private position high_goal_1 = new position(high_goal_general_position.getX(),high_goal_general_position.getY() ,high_goal_general_position.getAngleRadians());
    private position high_goal_2 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
    private position high_goal_3 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());

    private position wobble_goal_spot;
    private position second_wobble_goal = new position(27,-31,Math.toRadians(0));

    private position to_second_wobble_avoid_stack = new position(45,second_wobble_goal.getY(),Math.toRadians(0));
    private position on_way_to_second_wobble_spot = new position(50,-30,Math.toRadians(0));
    private position to_line_avoid_wobble_goals;
    private position line = new position(70,-25,Math.toRadians(180));
    private position shoot_starting_stack_position = high_goal_general_position;

    private position ring_stack = new position(34,-11.75,Math.toRadians(177));
    private position straight_behind_ring = new position(20,ring_stack.getY() - 2.5,Math.toRadians(177));
    private position infront_of_ring_stack = new position(high_goal_general_position.getX(),ring_stack.getY(),ring_stack.getAngleRadians());

    ArrayList<position> ring_path = new ArrayList<>();
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
    private long timeForWobbleDelivery = 850;//3 * 1000;
    private long timeOfWobblePlace = 0;
    private long time_between_shots = 200;
    private RingDetector.Height stack;

    @Override
    public void runOpMode() {

        //build path to the ring
        ring_path.add(straight_behind_ring);
        ring_path.add(ring_stack);


        // build path to make jerk occur
        high_goal_after_pickup_intentional_jerk.add(beyond_high_goal_spot);
        high_goal_after_pickup_intentional_jerk.add(high_goal_general_position);


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
                    high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() , powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 7, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 14, powershot_general_position.getAngleRadians() + Math.toRadians(5));



                    wobble_goal_spot = new position(59,5,Math.toRadians(180));
                    break;
                case ONE:

                    selectedShootingRPM = robot.powerShotSpeed;
                    selectedShootingTPM = robot.powerShotTicksPerMinute;
                    high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() , powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 7, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 14, powershot_general_position.getAngleRadians() + Math.toRadians(5));


                    wobble_goal_spot = new position(80.5,-15,Math.toRadians(-170));
                    break;
                case FOUR:


                    selectedShootingRPM = robot.normflywheelspeed;
                    selectedShootingTPM = robot.flywheelticksperminute;
                    high_goal_1 = new position(high_goal_general_position.getX(),high_goal_general_position.getY() ,high_goal_general_position.getAngleRadians());
                    high_goal_2 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());
                    high_goal_3 = new position(high_goal_general_position.getX(),high_goal_general_position.getY(),high_goal_general_position.getAngleRadians());


                    wobble_goal_spot = new position(104,3,Math.toRadians(-160));
                    second_wobble_goal = new position(27,-23.9,Math.toRadians(0));
                    break;
                default:


                    selectedShootingRPM = robot.powerShotSpeed;
                    selectedShootingTPM = robot.powerShotTicksPerMinute;
                    high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() , powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 7, powershot_general_position.getAngleRadians() + Math.toRadians(5));
                    high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 14, powershot_general_position.getAngleRadians() + Math.toRadians(5));


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
                    robot.shooter.setVelocity(selectedShootingTPM);
                    break;
                case SHOOT_FIRST_POWER_SHOT:
                    robot.shooter.setVelocity(selectedShootingTPM);
                    robot.goodDriveToPoint(high_goal_1);

                    if (((robot.shooter.getVelocity() / 28) * 60) > selectedShootingRPM - 100 && (System.currentTimeMillis() - (time_of_shot_arrival + 1000)) > time_between_shots && Math.abs(robot.yError) < 2) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        AUTO_STATE = auto_states.SHOOT_SECOND_POWER_SHOT;
                        time_of_shot_1 = System.currentTimeMillis();
                    }

                    break;
                case SHOOT_SECOND_POWER_SHOT:
                    robot.shooter.setVelocity(selectedShootingTPM);
                    robot.goodDriveToPoint(high_goal_2);

                    if (((robot.shooter.getVelocity() / 28) * 60) > selectedShootingRPM - 100 && (System.currentTimeMillis() - time_of_shot_1) > time_between_shots && Math.abs(robot.yError) < 2) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        AUTO_STATE = auto_states.SHOOT_THIRD_POWER_SHOT;
                        time_of_shot_2 = System.currentTimeMillis();
                    }
                    break;
                case SHOOT_THIRD_POWER_SHOT:
                    robot.shooter.setVelocity(selectedShootingTPM);
                    robot.goodDriveToPoint(high_goal_3);

                    if (((robot.shooter.getVelocity() / 28) * 60) > selectedShootingRPM - 100 && (System.currentTimeMillis() - time_of_shot_2) > time_between_shots && Math.abs(robot.yError) < 2) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);

                        if (stack.equals(ZERO)) {
                            AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE;
                        } else {
                            if (stack.equals(FOUR)) {
                                robot.ring_bumper.setPosition(robot.RING_BUMPER_OUT);
                            }
                            AUTO_STATE = auto_states.GO_INFRONT_OF_RING;
                        }
                    }

                    break;
                case GO_INFRONT_OF_RING:
                    robot.goodDriveToPoint(infront_of_ring_stack);
                    if (robot.robotPose.distanceToPose(infront_of_ring_stack) < 3) {
                        AUTO_STATE = auto_states.PICK_UP_SECOND_RING;
                    }
                    break;
                case PICK_UP_SECOND_RING:
                    robot.shooter.setVelocity(0);

                    robot.intake.setPower(1);
                    robot.goodDriveToPointDistanceControl(ring_stack,0.75);
                    if (robot.robotPose.distanceToPose(ring_stack) < 3) {
                        AUTO_STATE = auto_states.DRIVE_TO_SHOOTING_POSITION_AGAIN;
                    }
                    break;
                case DRIVE_TO_SHOOTING_POSITION_AGAIN:
                    robot.goodDriveToPoint(beyond_high_goal_spot);
                    if (robot.robotPose.distanceToPose(beyond_high_goal_spot) < 2.5) {
                        AUTO_STATE = auto_states.SHOOT_RING_AGAIN;
                    }
                    break;
                case JANK_JERK_THING_POG:
                    robot.shooter.setVelocity(selectedShootingTPM);

                    robot.goodDriveToPoint(high_goal_general_position);
                    if (robot.robotPose.distanceToPose(high_goal_general_position) < 2.5) {
                        AUTO_STATE = auto_states.SHOOT_RING_FOUR;
                        time_of_extra_shot = System.currentTimeMillis();
                    }
                    break;
                case SHOOT_RING_AGAIN:
                    if (robot.robotPose.distanceToPose(shoot_starting_stack_position) < 3) {
                        robot.shooter.setVelocity(robot.secondSpeedflywheelticksperminute);
                    }

                    robot.goodDriveToPoint(shoot_starting_stack_position);

                    if (((robot.shooter.getVelocity() / 28) * 60) > robot.secondSpeed - 100 && (System.currentTimeMillis() - time_of_extra_shot) > time_between_shots && robot.robotPose.distanceToPose(shoot_starting_stack_position) < 2) {
                        robot.drive.STOP();
                        // shoot one or three times
                        double actuation_times = 1;
                        if (stack.equals(RingDetector.Height.FOUR)) {
                            actuation_times = 3;
                        }
                        for (int i = 0; i < actuation_times; i++) {
                            robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                            sleep(shooter_actuation_time);
                            robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                            sleep(shooter_actuation_time / 2);
                        }


                        if (stack.equals(RingDetector.Height.ONE)) {
                            AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE;
                        } else {
                            AUTO_STATE = auto_states.PICK_UP_FOURTH_RING;
                        }
                    }

                    break;
                case PICK_UP_FOURTH_RING:
                    robot.shooter.setVelocity(0);
                    robot.goodDriveToPoint(straight_behind_ring);
                    if (robot.robotPose.distanceToPose(straight_behind_ring) < 2.5) {
                        AUTO_STATE = auto_states.DRIVE_TO_SHOOTING_POSITION_FOUR;
                    }
                    break;
                case DRIVE_TO_SHOOTING_POSITION_FOUR:
                    robot.shooter.setVelocity(selectedShootingTPM);

                    robot.goodDriveToPoint(shoot_starting_stack_position);
                    if (robot.robotPose.distanceToPose(shoot_starting_stack_position) < 2.5) {
                        AUTO_STATE = auto_states.SHOOT_RING_FOUR;
                        time_of_extra_shot = System.currentTimeMillis();
                    }

                    break;
                case SHOOT_RING_FOUR:

                    if (robot.robotPose.distanceToPose(high_goal_general_position) < 3) {
                        robot.shooter.setVelocity(robot.secondSpeedflywheelticksperminute);
                    }

                    robot.goodDriveToPoint(high_goal_general_position);

                    if (((robot.shooter.getVelocity() / 28) * 60) > robot.secondSpeed - 100 && (System.currentTimeMillis() - time_of_extra_shot) > time_between_shots && robot.robotPose.distanceToPose(high_goal_general_position) < 2) {
                        robot.drive.STOP();

                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2);

                        AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE;

                    }

                    break;

                case DRIVE_TO_WOBBLE_ZONE:
                    robot.ring_bumper.setPosition(robot.RING_BUMPER_IN);
                    robot.intake.setPower(0);

                    if (robot.robotPose.distanceToPose(new position(wobble_goal_spot.getX() + 5,wobble_goal_spot.getY() + 8,wobble_goal_spot.getAngleRadians())) < 3) {
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
                    robot.goodDriveToPointWithMaxSpeed(second_wobble_goal,0.67);
                    if (robot.robotPose.distanceToPose(second_wobble_goal) < 2) {
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
                        arm_position = robot.LIFT_IN;

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
                    arm_position = robot.LIFT_IN;

                    robot.goodDriveToPoint(to_line_avoid_wobble_goals);
                    if (robot.robotPose.distanceToPose(to_line_avoid_wobble_goals) < 4) {
                        AUTO_STATE = auto_states.PARK_ON_LINE;

                        timeOfWobblePlace = System.currentTimeMillis();

                    }
                    break;


                case PARK_ON_LINE:
                    arm_position = robot.LIFT_IN;

                    robot.shooter.setVelocity(0);

                    robot.wrist.setPosition(robot.WRIST_IN);
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    robot.goodDriveToPoint(line);
                    break;
            }
            robot.driveArmToSetpoint(arm_position);
            telemetry.addData("shooter velo", (robot.shooter.getVelocity() / 28) * 60);
            telemetry.addData("current x", robot.robotPose.getX());
            telemetry.addData("current y", robot.robotPose.getY());
            telemetry.addData("heading error deg", Math.toDegrees(robot.headingError));
            telemetry.addData("current auto state",AUTO_STATE);
            telemetry.addData("stack is: ",stack);
            telemetry.update();

        }
    }

}
