package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

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
    double first_pixel_average_measurement = 0;
    double second_pixel_average_measurement = 0;
    double third_pixel_average_measurement = 0;
    double fourth_pixel_average_measurement = 0;
    double fith_pixel_average_measurement = 0;
    double pixel_average = 0;
    private double setpoint = 0;
    private position SHOOTING_POSITION = new position(24,0,Math.toRadians(172));

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);

        robot = new RobotClass();
        robot.init(hardwareMap);
        robot.setBreak();

        telemetry.addData("ready to start","Press play!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            long loopstart = System.currentTimeMillis();
            roadrunnerOdometry.updatePoseEstimate();

            Pose2d pose = roadrunnerOdometry.getPoseEstimate();

            robot.robotPose.setPose2dRoadRunner(pose);
            boolean intakeButtonPress = gamepad1.x;
            boolean turnOnShooter = gamepad1.b;
            boolean turnOnShooterSlow = gamepad1.dpad_right;
            boolean shootRing = gamepad1.y;
            boolean wobble_state_advance = gamepad1.dpad_up;
            boolean wobble_state_backward = gamepad1.dpad_down;


            if (!sticksNotOutsideThreshold(0.8) && SHOOTER_MOTOR_STATE.equals(shooterMotorState.OFF)) {
                robot.intake.setPower(1);
            } else {
                robot.intake.setPower(0);
            }

            switch (DRIVING_STATE) {

                case HUMAN_CONTROL:
                    /*
                    if (gamepad1.left_bumper && Math.abs(vision_error) > robot.VISION_THRESHOLD) {
                        robot.robotRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x + (vision_error * 0.002));

                     */
                    if (gamepad1.left_bumper) {
                        Pose2d estimate = roadrunnerOdometry.getPoseEstimate();
                        SHOOTING_POSITION.setPose2dRoadRunner(estimate);
                    } else {
                        robot.robotRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
                    }
                    if (gamepad1.a) {
                        DRIVING_STATE = drivingStates.AUTO_AIM;
                    } else if (gamepad1.right_bumper) {
                        DRIVING_STATE = drivingStates.POWER_SHOT_AIM;
                    }
                    break;
                case AUTO_AIM:

                    robot.goodDriveToPoint(SHOOTING_POSITION);
                    if (sticksNotOutsideThreshold(0.1)) {
                        DRIVING_STATE = drivingStates.HUMAN_CONTROL;
                    }
                    break;
                case POWER_SHOT_AIM:
                    robot.goodDriveToPoint(new position(SHOOTING_POSITION.getX()+5,SHOOTING_POSITION.getY()-20,SHOOTING_POSITION.getAngleRadians()));
                    if (sticksNotOutsideThreshold(0.1)) {
                        DRIVING_STATE = drivingStates.HUMAN_CONTROL;
                    }
                    break;
                case STOPPED:
                    robot.drive.STOP();
                    telemetry.addData("bad thing happened","restart robot controller");
                    break;
            }

            switch (SHOOTER_MOTOR_STATE) {
                case OFF:
                    if (turnOnShooter && (PREVIOUS_SHOOTER_BUTTON_STATE != turnOnShooter)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.ON;
                    } else if (turnOnShooterSlow && (PREVIOUS_SHOOTER_SLOW_BUTTON_STATE != turnOnShooterSlow)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.REDUCED_SPEED;
                    }
                    robot.shooter.setVelocity(0);
                    robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
                    break;
                case ON:
                    if (turnOnShooter && (PREVIOUS_SHOOTER_BUTTON_STATE != turnOnShooter)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.OFF;
                    } else if (turnOnShooterSlow && (PREVIOUS_SHOOTER_SLOW_BUTTON_STATE != turnOnShooterSlow)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.REDUCED_SPEED;
                    }
                    robot.shooter.setVelocity(robot.flywheelticksperminute);
                    break;
                case REDUCED_SPEED:
                    robot.shooter.setVelocity(robot.powerShotTicksPerMinute);
                    if (turnOnShooter && (PREVIOUS_SHOOTER_BUTTON_STATE != turnOnShooter)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.OFF;
                    } else if (turnOnShooterSlow && (PREVIOUS_SHOOTER_SLOW_BUTTON_STATE != turnOnShooterSlow)) {
                        SHOOTER_MOTOR_STATE = shooterMotorState.OFF;
                    }

                    break;
            }
            if (shootRing) {
                robot.shooterArm.setPosition(robot.SHOOTER_ARM_OUT);
            } else {
                robot.shooterArm.setPosition(robot.SHOOTER_ARM_IN);
            }


            switch (WOBBLE_LIFT_STATE) {
                case IN:
                    if (wobble_state_advance && !PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MAX_HEIGHT;
                    }
                    setpoint = robot.LIFT_IN;
                    robot.wrist.setPosition(robot.WRIST_IN);
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    break;
                case MAX_HEIGHT:
                    if (wobble_state_advance && !PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MID;
                    }
                    else if (wobble_state_backward && !PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.IN;
                    }
                    robot.wrist.setPosition(robot.WRIST_IN);
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    setpoint = robot.LIFT_MAX;
                    break;
                case MID:
                    if (wobble_state_advance && !PREVIOUS_WOBBLE_LIFT_ADVANCE_FORWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.DOWN;
                    }
                    else if (wobble_state_backward && !PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MAX_HEIGHT;
                    }
                    robot.wrist.setPosition(robot.WRIST_FOR_PLACING_OUTSIDE);
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                    setpoint = robot.LIFT_MID;
                    break;
                case DOWN:
                    if (wobble_state_backward && !PREVIOUS_WOBBLE_LIFT_ADVANCE_BACKWARD_BUTTON_STATE) {
                        WOBBLE_LIFT_STATE = wobbleLiftstates.MID;
                        robot.claw.setPosition(robot.CLAW_CLOSED);
                        sleep(500);

                    }
                    robot.wrist.setPosition(robot.WRIST_FOR_GRAB);
                    robot.claw.setPosition(robot.CLAW_OPEN);
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

            telemetry.addData("robot X",robot.robotPose.getX());
            telemetry.addData("robot Y",robot.robotPose.getY());
            telemetry.addData("roadrunner angle",robot.robotPose.getAngleDegrees());
            telemetry.addData("loop update time: ",System.currentTimeMillis() - loopstart);
            telemetry.addData("shooter arm state: ", SHOOTER_ARM_STATE);
            telemetry.addData("shootRing ",shootRing);
            telemetry.addData("wobble arm",robot.lift.getCurrentPosition());
            telemetry.addData("shooter vleo",(robot.shooter.getVelocity() / 28) * 60);
            telemetry.update();
        }

    }

    public boolean sticksNotOutsideThreshold(double threshold) {
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
}
