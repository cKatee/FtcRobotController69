package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@Autonomous
public class RedLeft extends auto {
    private auto_states AUTO_STATE = auto_states.START;
    private position start_position = new position(0,0,Math.toRadians(0));
    private position power_shot_general_position = new position(20,0,Math.toRadians(-160));
    private position power_shot_1 = new position(power_shot_general_position.getX(),power_shot_general_position.getY(),Math.toRadians(-160));
    private position power_shot_2 = new position(power_shot_general_position.getX(),power_shot_general_position.getY(),Math.toRadians(-155));
    private position power_shot_3 = new position(power_shot_general_position.getX(),power_shot_general_position.getY(),Math.toRadians(-150));
    private position wobble_goal_spot = new position(78,-15,Math.toRadians(170));
    private position line = new position(75,-25,Math.toRadians(0));
    private double arm_position = 0;
    private long timeOfWobbleDelivery1Start = 0;

    /**
     * timersss
     */

    // time in milliseconds allowed for the robot to turn towards the target
    private long time_for_shot_adjust = 400;
    // time in milliseconds for the servo to push the disk
    private long shooter_actuation_time = 400;
    private long timeForWobbleDelivery = 3 * 1000;
    private long timeOfWobblePlace = 0;


    @Override
    public void runOpMode() {
        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);

        initialize();
        while (opModeIsActive()) {

            roadrunnerOdometry.updatePoseEstimate();
            Pose2d position_estimate = roadrunnerOdometry.getPoseEstimate();
            robot.robotPose.setPose2dRoadRunner(position_estimate);
            switch (AUTO_STATE) {
                case START:
                    robot.goodDriveToPoint(start_position);
                    if (robot.robotPose.distanceToPose(start_position) < 4) {
                        AUTO_STATE = auto_states.DRIVE_TO_POWER_SHOT;
                    }
                    break;
                case DRIVE_TO_POWER_SHOT:
                    if (robot.robotPose.distanceToPose(power_shot_general_position) < 3 && ((robot.shooter.getVelocity() / 28) * 60) > 5090) {
                        AUTO_STATE = auto_states.SHOOT_FIRST_POWER_SHOT;
                    }
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    robot.goodDriveToPoint(power_shot_general_position);
                    break;
                case SHOOT_FIRST_POWER_SHOT:
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    if (((robot.shooter.getVelocity() / 28) * 60) > 5090 && Math.abs(robot.headingError) < Math.toRadians(3)) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2);

                        AUTO_STATE = auto_states.SHOOT_SECOND_POWER_SHOT;
                    }
                    robot.turnToAngle(power_shot_1);
                    break;
                case SHOOT_SECOND_POWER_SHOT:
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    if (((robot.shooter.getVelocity() / 28) * 60) > 5090 && Math.abs(robot.headingError) < Math.toRadians(3)) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2 );
                        AUTO_STATE = auto_states.SHOOT_THIRD_POWER_SHOT;
                    }
                    robot.turnToAngle(power_shot_2);
                    break;
                case SHOOT_THIRD_POWER_SHOT:
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    if (((robot.shooter.getVelocity() / 28) * 60) > 5090 && Math.abs(robot.headingError) < Math.toRadians(3)) {
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
                        sleep(shooter_actuation_time);
                        robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                        sleep(shooter_actuation_time / 2);

                        AUTO_STATE = auto_states.DRIVE_TO_WOBBLE_ZONE;
                    }

                    robot.turnToAngle(power_shot_3);
                    break;
                case DRIVE_TO_WOBBLE_ZONE:
                    if (robot.robotPose.distanceToPose(wobble_goal_spot) < 3) {
                        AUTO_STATE = auto_states.PLACE_WOBBLE_GOAL;
                        timeOfWobbleDelivery1Start = System.currentTimeMillis();
                    }
                    robot.shooter.setVelocity(0);
                    arm_position = robot.LIFT_MID;
                    robot.goodDriveToPoint(wobble_goal_spot);
                    break;
                case PLACE_WOBBLE_GOAL:
                    if (System.currentTimeMillis() - timeOfWobbleDelivery1Start > 1000) {
                        robot.claw.setPosition(robot.CLAW_OPEN);
                    }
                    if ((timeForWobbleDelivery + timeOfWobbleDelivery1Start) < System.currentTimeMillis()) {
                        AUTO_STATE = auto_states.PARK_ON_LINE;
                        // ensure we go back 10x from where we are so we dont make the wobble goal die
                        line.setX(position_estimate.getX() - 10);
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
            telemetry.addData("shooter velo",(robot.shooter.getVelocity() / 28) * 60);
            telemetry.addData("current x",robot.robotPose.getX());
            telemetry.addData("current y",robot.robotPose.getY());
            telemetry.addData("angle",robot.getAngleIMU());
            telemetry.addData("current auto state",AUTO_STATE);
            telemetry.update();

        }
    }

}
