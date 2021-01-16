package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.autonomous.RedLeftRefactored;
import org.firstinspires.ftc.teamcode.controllers.LQRMotionProfiledPoseStabalizationController;
import org.firstinspires.ftc.teamcode.controllers.MotionProfiledPoseStablization;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@TeleOp
public class teleop extends LinearOpMode {

    private RobotClass robot;
    OpenCvCamera webcam;

    private drivingStates DRIVING_STATE = drivingStates.HUMAN_CONTROL;
    private intakeStates INTAKE_STATE = intakeStates.OFF;
    private shooterMotorState SHOOTER_MOTOR_STATE = shooterMotorState.OFF;
    private shooterArmState SHOOTER_ARM_STATE = shooterArmState.IN;
    private wobbleLiftstates WOBBLE_LIFT_STATE = wobbleLiftstates.IN;
    private boolean PREVIOUS_INTAKE_BUTTON_STATE = false;
    private boolean PREVIOUS_SHOOTER_BUTTON_STATE = false;
    private boolean PREVIOUS_SHOOTER_SLOW_BUTTON_STATE = false;
    private boolean PREVIOUS_SHOOTER_ARM_BUTTON_STATE = false;
    private boolean PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE = false;
    private boolean PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE = false;
    private boolean shooter_is_actuated = false;
    private boolean last_shooter_actuated = false;
    private double time_of_shooter_state_change = 0;
    private position powershot_general_position = new position(63,-34,Math.toRadians(-174));

    private position high_goal_1 = new position(powershot_general_position.getX(), powershot_general_position.getY() - 5.5, powershot_general_position.getAngleRadians() + Math.toRadians(5));
    private position high_goal_2 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 1, powershot_general_position.getAngleRadians() + Math.toRadians(5));
    private position high_goal_3 = new position(powershot_general_position.getX(), powershot_general_position.getY() + 9, powershot_general_position.getAngleRadians() + Math.toRadians(5));

    private boolean PREVIOUS_INTAKE_STATE = false;
    private double time_of_intake_off = 0;
    // time in milliseconds we wait before turning off the intake so we still get the ring
    private final double INTAKE_OFF_DELAY = 500;
    double first_pixel_average_measurement = 0;
    double second_pixel_average_measurement = 0;
    double third_pixel_average_measurement = 0;
    double fourth_pixel_average_measurement = 0;
    double fith_pixel_average_measurement = 0;
    double pixel_average = 0;
    private double setpoint = 0;
    private position SHOOTING_POSITION = new position(59.04603135856674,-7.18,Math.toRadians(189.55));
    private ElapsedTime timer = new ElapsedTime();

    private double intake_speed = 0;
    private double last_intake_speed = 100000;

    private double shooter_velocity = 0;
    private double last_shooter_velocity = 100000;

    private powerShotAimStates power_shot_automation_state = powerShotAimStates.NOT_BEGUN;

    // time in between shots
    private double shooter_actuation_time_for_multi_shots_to_be_selected = 100000;
    @Override
    public void runOpMode() throws InterruptedException {

        LQRMotionProfiledPoseStabalizationController MotionProfiledController = new LQRMotionProfiledPoseStabalizationController(robot);
        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        roadrunnerOdometry.setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));
        robot = new RobotClass();
        robot.init(hardwareMap);
        robot.setBreak();
        telemetry.addData("ready to start","Press play!");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {


            timer.reset();
            roadrunnerOdometry.updatePoseEstimate();

            Pose2d pose = roadrunnerOdometry.getPoseEstimate();

            robot.robotPose.setPose2dRoadRunner(pose);
            boolean intakeReverseButtonPress = gamepad1.x;
            boolean turnOnShooter = gamepad1.b;
            boolean turnOnShooterSlow = gamepad1.dpad_right;
            boolean shootRing = gamepad1.y;
            boolean wobble_state_advance = gamepad1.dpad_up;
            boolean wobble_state_backward = gamepad1.dpad_down;
            boolean intake_on = !sticksOutsideThreshold(0.8) && SHOOTER_MOTOR_STATE.equals(shooterMotorState.OFF);


            if (intake_on) {
                if (intakeReverseButtonPress) {
                    intake_speed = -1;
                } else {
                    intake_speed = 1;
                }
            } else {
                 intake_speed = 1;
            }

            if (intake_speed != last_intake_speed) {
                robot.intake.setPower(intake_speed);
            }
            last_intake_speed = intake_speed;


            switch (DRIVING_STATE) {

                case HUMAN_CONTROL:
                    /*
                    if (gamepad1.left_bumper && Math.abs(vision_error) > robot.VISION_THRESHOLD) {
                        robot.robotRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x + (vision_error * 0.002));

                     */
                    if ((shootRing) && !sticksOutsideThreshold(0.0)) {
                        Pose2d estimate = roadrunnerOdometry.getPoseEstimate();
                        SHOOTING_POSITION.setPose2dRoadRunner(estimate);
                    } else if (gamepad1.left_bumper){

                        roadrunnerOdometry.setPoseEstimate(new Pose2d(130.3,-48.42,Math.toRadians(180)));

                    } else {
                        robot.robotRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                    }
                    if (gamepad1.a) {
                        DRIVING_STATE = drivingStates.AUTO_AIM;
                    } else if (gamepad1.dpad_left) {
                        DRIVING_STATE = drivingStates.POWER_SHOT_AIM;
                    }
                    break;
                case AUTO_AIM:

                    robot.goodDriveToPoint(SHOOTING_POSITION);
                    if (sticksOutsideThreshold(0.1)) {
                        DRIVING_STATE = drivingStates.HUMAN_CONTROL;
                    }
                    break;
                case POWER_SHOT_AIM:
                    if (sticksOutsideThreshold(0.1)) {
                        DRIVING_STATE = drivingStates.HUMAN_CONTROL;
                    }
                    switch (power_shot_automation_state) {
                        case NOT_BEGUN:
                            break;
                        case GOING_TO_FIRST_POWERSHOT:
                            if (MotionProfiledController.goToPositionFast(high_goal_1,2.3)) {

                            }
                            break;
                        case SHOOTING_FIRST_POWER_SHOT:
                            break;
                        case GOING_TO_SECOND_POWER_SHOT:
                            break;
                        case SHOOTING_SECOND_POWER_SHOT:;
                            break;
                        case GOING_TO_THIRD_POWER_SHOT:
                            break;
                        case SHOOTING_THIRD_POWER_SHOT:
                            break;

                    }


                    break;
                case STOPPED:
                    robot.drive.STOP();
                    telemetry.addData("bad thing happened","restart robot controller");
                    telemetry.update();
                    break;
            }

            switch (SHOOTER_MOTOR_STATE) {
                case OFF:
                    if (turnOnShooter && (PREVIOUS_SHOOTER_BUTTON_STATE != turnOnShooter)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.ON;
                    } else if (turnOnShooterSlow && (PREVIOUS_SHOOTER_SLOW_BUTTON_STATE != turnOnShooterSlow)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.REDUCED_SPEED;
                    }
                    shooter_velocity = 0;

                    if (shooter_velocity != last_shooter_velocity) {
                        robot.shooter.setPower(shooter_velocity);
                    }

                    last_shooter_velocity = shooter_velocity;
                    break;
                case ON:
                    if (turnOnShooter && (PREVIOUS_SHOOTER_BUTTON_STATE != turnOnShooter)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.OFF;
                    } else if (turnOnShooterSlow && (PREVIOUS_SHOOTER_SLOW_BUTTON_STATE != turnOnShooterSlow)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.REDUCED_SPEED;
                    }
                    //robot.setShooterArmLynxOptimized(robot.SHOOTER_ARM_IN);

                    shooter_velocity = robot.flywheelticksperminute;

                    if (shooter_velocity != last_shooter_velocity) {
                        robot.shooter.setVelocity(shooter_velocity);
                    }
                    last_shooter_velocity = shooter_velocity;

                    break;
                case REDUCED_SPEED:
                    shooter_velocity = robot.powerShotTicksPerMinute;
                    robot.setShooterArmLynxOptimized(robot.SHOOTER_ARM_IN);
                    if (turnOnShooter && (PREVIOUS_SHOOTER_BUTTON_STATE != turnOnShooter)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.OFF;
                    } else if (turnOnShooterSlow && (PREVIOUS_SHOOTER_SLOW_BUTTON_STATE != turnOnShooterSlow)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.OFF;
                    }

                    if (shooter_velocity != last_shooter_velocity) {
                        robot.shooter.setVelocity(shooter_velocity);
                    }

                    last_shooter_velocity = shooter_velocity;
                    break;
            }
            if (shootRing) {

                if (SHOOTER_MOTOR_STATE.equals(shooterMotorState.REDUCED_SPEED)) {
                    if (shootRing) {
                        robot.setShooterArmLynxOptimized(robot.SHOOTER_ARM_OUT);
                    } else {
                        robot.setShooterArmLynxOptimized(robot.SHOOTER_ARM_IN);
                    }
                }
                else {
                    if (shooter_is_actuated) {
                        shooter_actuation_time_for_multi_shots_to_be_selected = 200;
                    } else {
                        shooter_actuation_time_for_multi_shots_to_be_selected = 140;
                    }
                    if (System.currentTimeMillis() - time_of_shooter_state_change >= shooter_actuation_time_for_multi_shots_to_be_selected) {
                        time_of_shooter_state_change = System.currentTimeMillis();
                        shooter_is_actuated = !shooter_is_actuated;
                    }

                    if (shooter_is_actuated) {
                        robot.setShooterArmLynxOptimized(robot.SHOOTER_ARM_OUT);
                    } else {
                        robot.setShooterArmLynxOptimized(robot.SHOOTER_ARM_IN);

                    }
                }



            } else {
                shooter_is_actuated = false;
                robot.setShooterArmLynxOptimized(robot.SHOOTER_ARM_IN);
            }


            switch (WOBBLE_LIFT_STATE) {
                case IN:
                    if (wobble_state_advance && !PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MAX_HEIGHT;
                    }
                    setpoint = robot.LIFT_IN;
                    robot.setWristLynxOptimized(robot.WRIST_IN);
                    robot.setClawLynxOptimized(robot.CLAW_CLOSED);
                    break;
                case MAX_HEIGHT:
                    if (wobble_state_advance && !PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MID;
                    }
                    else if (wobble_state_backward && !PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.IN;
                    }
                    robot.setWristLynxOptimized(robot.WRIST_IN);
                    robot.setClawLynxOptimized(robot.CLAW_CLOSED);
                    setpoint = robot.LIFT_MAX;
                    break;
                case MID:
                    if (wobble_state_advance && !PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.DOWN;
                    }
                    else if (wobble_state_backward && !PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MAX_HEIGHT;
                    }
                    robot.setWristLynxOptimized(robot.WRIST_FOR_PLACING_OUTSIDE);
                    robot.setClawLynxOptimized(robot.CLAW_CLOSED);
                    setpoint = robot.LIFT_MID;
                    break;
                case DOWN:
                    if (wobble_state_backward && !PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MID;
                        robot.setClawLynxOptimized(robot.CLAW_CLOSED);
                        sleep(500);

                    }
                    robot.setWristLynxOptimized(robot.WRIST_FOR_GRAB);
                    robot.setClawLynxOptimized(robot.CLAW_OPEN);
                    setpoint = robot.LIFT_DOWN;
                    break;

            }

            PREVIOUS_SHOOTER_BUTTON_STATE = turnOnShooter;
            PREVIOUS_SHOOTER_SLOW_BUTTON_STATE = turnOnShooterSlow;
            PREVIOUS_SHOOTER_ARM_BUTTON_STATE = gamepad1.y;
            PREVIOUS_INTAKE_BUTTON_STATE = gamepad1.a;
            PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE = wobble_state_advance;
            PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE = wobble_state_backward;

            robot.driveArmToSetpoint(setpoint);


            /*

            telemetry.addData("robot X",robot.robotPose.getX());
            telemetry.addData("robot Y",robot.robotPose.getY());
            telemetry.addData("roadrunner angle",robot.robotPose.getAngleDegrees());
            telemetry.addData("shooter arm state: ", SHOOTER_ARM_STATE);
            telemetry.addData("shootRing ",shootRing);
            telemetry.addData("wobble arm",robot.lift.getCurrentPosition());
            telemetry.addData("shooter vleo",(robot.shooter.getVelocity() / 28) * 60);
            telemetry.update();

            */

            System.out.println("odo x:" + robot.robotPose.getX() + " odo y: " + robot.robotPose.getY() + " angle odo: " + robot.robotPose.getAngleDegrees());
            System.out.println("loop time" + timer.milliseconds());
        }

    }

    public boolean sticksOutsideThreshold(double threshold) {
        boolean SticksOutsideThreshold = false;

        if (Math.abs(gamepad1.left_stick_y) > threshold) {
            SticksOutsideThreshold = true;
        }
        if (Math.abs(gamepad1.left_stick_x) > threshold) {
            SticksOutsideThreshold = true;
        }
        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            SticksOutsideThreshold = true;
        }

        return SticksOutsideThreshold;
    }

    public enum drivingStates {
        HUMAN_CONTROL,
        AUTO_AIM,
        POWER_SHOT_AIM,
        STOPPED
    }
    public enum intakeStates {
        OFF,
        FORWARD,
    }
    public enum shooterMotorState {
        ON,
        REDUCED_SPEED,
        OFF,
    }
    public enum shooterArmState {
        IN,
        OUT
    }
    public enum wobbleLiftstates {
        IN,
        MAX_HEIGHT,
        MID,
        DOWN
    }
    public enum powerShotAimStates {
        NOT_BEGUN,
        GOING_TO_FIRST_POWERSHOT,
        GOING_TO_SECOND_POWER_SHOT,
        GOING_TO_THIRD_POWER_SHOT,
        SHOOTING_FIRST_POWER_SHOT,
        SHOOTING_SECOND_POWER_SHOT,
        SHOOTING_THIRD_POWER_SHOT

    }

}
