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
    private position power_shot_so_we_miss_starting_stack = new position(60,6,Math.toRadians(180));
    private position power_shot_general_position = new position(52,-9,Math.toRadians(176));
    private position power_shot_1 = new position(power_shot_general_position.getX(),power_shot_general_position.getY() ,power_shot_general_position.getAngleRadians());
    private position power_shot_2 = new position(power_shot_general_position.getX(),power_shot_general_position.getY(),power_shot_general_position.getAngleRadians());
    private position power_shot_3 = new position(power_shot_general_position.getX(),power_shot_general_position.getY(),power_shot_general_position.getAngleRadians());
    private position wobble_goal_spot;
    private position on_way_to_second_wobble_spot = new position(50,-30,Math.toRadians(0));
    private position second_wobble_goal = new position(27,-23,Math.toRadians(0));
    private position line = new position(70,0,Math.toRadians(0));

    private double arm_position = 0;
    private long timeOfWobbleDelivery1Start = 0;


    private long time_of_shot_1 = 0;
    private long time_of_shot_2 = 0;
    private long time_of_shot_arrival = 0;
    /**
     * timersss
     */

    // time in milliseconds allowed for the robot to turn towards the target
    private long time_for_shot_adjust = 400;
    // time in milliseconds for the servo to push the disk
    private long shooter_actuation_time = 400;
    private long timeForWobbleDelivery = 3 * 1000;
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
                    wobble_goal_spot = new position(63,6,Math.toRadians(170));
                    break;
                case ONE:
                    wobble_goal_spot = new position(78,-15,Math.toRadians(170));
                    break;
                case FOUR:
                    wobble_goal_spot = new position(100,6,Math.toRadians(180));
                    break;
                default:
                    // default to zone B
                    wobble_goal_spot = new position(78,-15,Math.toRadians(170));
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
                    robot.goodDriveToPoint(power_shot_so_we_miss_starting_stack);
                    if (robot.robotPose.distanceToPose(power_shot_so_we_miss_starting_stack) < 2.5) {
                        AUTO_STATE = auto_states.DRIVE_TO_POWER_SHOT;
                    }
                    break;
                case DRIVE_TO_POWER_SHOT:
                    if (robot.robotPose.distanceToPose(power_shot_general_position) < 1.5 && ((robot.shooter.getVelocity() / 28) * 60) > 3700) {
                        AUTO_STATE = auto_states.SHOOT_FIRST_POWER_SHOT;
                        robot.drive.STOP();
                        time_of_shot_arrival = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    robot.goodDriveToPoint(power_shot_general_position);
                    break;
                case SHOOT_FIRST_POWER_SHOT:
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    robot.goodDriveToPoint(power_shot_1);


                    if (((robot.shooter.getVelocity() / 28) * 60) > 3700 && (System.currentTimeMillis() - time_of_shot_arrival) > time_between_shots) {
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
                    robot.goodDriveToPoint(power_shot_2);

                    if (((robot.shooter.getVelocity() / 28) * 60) > 3700 && (System.currentTimeMillis() - time_of_shot_1) > time_between_shots) {
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
                    robot.goodDriveToPoint(power_shot_3);

                    if (((robot.shooter.getVelocity() / 28) * 60) > 3700 && (System.currentTimeMillis() - time_of_shot_2) > time_between_shots) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2);

                        AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE;
                    }

                    break;
                case DRIVE_TO_WOBBLE_ZONE:
                    if (robot.robotPose.distanceToPose(wobble_goal_spot) < 4) {
                        AUTO_STATE = auto_states.PLACE_WOBBLE_GOAL;
                        timeOfWobbleDelivery1Start = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_MID;
                    robot.goodDriveToPoint(wobble_goal_spot);
                    break;
                case PLACE_WOBBLE_GOAL:
                    if (System.currentTimeMillis() - timeOfWobbleDelivery1Start > 700) {
                        robot.claw.setPosition(robot.CLAW_OPEN);
                    }
                    if ((timeForWobbleDelivery + timeOfWobbleDelivery1Start) < System.currentTimeMillis()) {
                        AUTO_STATE = auto_states.GO_TO_SECOND_WOBBLE_GOAL;
                        // ensure we go back 10x from where we are so we dont make the wobble goal die
                        timeOfWobblePlace = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_DOWN;
                    robot.wrist.setPosition(robot.WRIST_FOR_PLACING_OUTSIDE);
                    robot.wrist.setPosition(robot.WRIST_FOR_GRAB);
                    robot.goodDriveToPoint(wobble_goal_spot);
                    break;
                case GO_TO_SECOND_WOBBLE_GOAL:
                    robot.goodDriveToPointWithMaxSpeed(second_wobble_goal,0.7);
                    if (robot.robotPose.distanceToPose(second_wobble_goal) < 1) {
                        AUTO_STATE = auto_states.GRAB_SECOND_WOBBLE_GOAL;
                    }
                    break;
                case GRAB_SECOND_WOBBLE_GOAL:
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    sleep(600);
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
                    if (robot.robotPose.distanceToPose(wobble_goal_spot) < 4) {
                        AUTO_STATE = auto_states.PLACE_WOBBLE_GOAL_2;
                        timeOfWobbleDelivery1Start = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_MID;
                    robot.goodDriveToPoint(wobble_goal_spot);
                    break;

                case PLACE_WOBBLE_GOAL_2:
                    if (System.currentTimeMillis() - timeOfWobbleDelivery1Start > 700) {
                        robot.claw.setPosition(robot.CLAW_OPEN);
                    }
                    if ((timeForWobbleDelivery + timeOfWobbleDelivery1Start) < System.currentTimeMillis()) {
                        AUTO_STATE = auto_states.PARK_ON_LINE;
                        // ensure we go back 10x from where we are so we dont make the wobble goal die
                        timeOfWobblePlace = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_DOWN;
                    robot.wrist.setPosition(robot.WRIST_FOR_PLACING_OUTSIDE);
                    robot.wrist.setPosition(robot.WRIST_FOR_GRAB);
                    robot.goodDriveToPoint(wobble_goal_spot);
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
