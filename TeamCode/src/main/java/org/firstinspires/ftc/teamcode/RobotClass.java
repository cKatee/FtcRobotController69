package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.geometry.position;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.GlutenCode.utils.AngleWrap;


public class RobotClass {

    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    public DcMotor intake;
    public DcMotorEx shooter;
    public DcMotor lift;
    public Servo wrist;
    public Servo claw;
    public Servo shooterArm;

    public final double TAU = Math.PI * 2;

    public static double xPower = 0;
    public static double yPower = 0;
    public static double turnPower = 0;

    // time that the last run of the go to point method was run for integration and derivation
    private double timeOfLastupdate = 0;
    private double last_error_x = 0;
    private double last_error_y = 0;
    private double last_error_angle = 0;
    private double track_width_meters = 0.328008996;
    private double adjacent_width_meters = 0.3106274204;

    private double wheel_pose_x = track_width_meters / 2;
    private double wheel_pose_y = adjacent_width_meters / 2;


    public static final double VISION_TARGET_PIXEL = 320 / 2;
    public static final double VISION_THRESHOLD = 70;
    public Translation2d FrontLeftPose = new Translation2d(wheel_pose_x,wheel_pose_y);
    public Translation2d FrontRightPose = new Translation2d(wheel_pose_x,-wheel_pose_y);
    public Translation2d BackLeftPose = new Translation2d(-wheel_pose_x,wheel_pose_y);
    public Translation2d BackRightPose = new Translation2d(-wheel_pose_x,-wheel_pose_y);

    public static double frontLeftPower = 0;
    public static double frontRightPower = 0;
    public static double backLeftPower = 0;
    public static double backRightPower = 0;
    public static double headingError = 0;

    private final double liftKp = 0.005;
    private final double liftKi = 0.000000015;
    private final double liftKd = 0.4;

    public final double LIFT_IN = 13;
    
    public final double LIFT_MAX = 359;
    public final double LIFT_MID = 400;
    public final double LIFT_DOWN = 900;

    /*
    public final double LIFT_MAX = LIFT_IN;
    public final double LIFT_MID = LIFT_IN;
    public final double LIFT_DOWN = LIFT_IN;

     */

    public final double CLAW_CLOSED = 0.8;
    public final double CLAW_OPEN = 0.2;


    public final double WRIST_FOR_AUTO_INIT = 0.5;
    public final double WRIST_IN = 1;
    public final double WRIST_FOR_PLACING_OUTSIDE = 0;
    public final double WRIST_FOR_GRAB = 0.2;
    public final double WRIST_FOR_SHOOTING = WRIST_IN;

    public final double SHOOTER_ARM_IN = 0.48;
    public final double SHOOTER_ARM_OUT = 0.8;
    public final double normflywheelspeed = 4500;
    public final double flywheelticksperminute = (normflywheelspeed * 28) / 60;
    public final double secondSpeed = 4500;
    public final double secondSpeedflywheelticksperminute = (secondSpeed * 28) / 60;

    public final double powerShotSpeed = 3600;
    public final double powerShotTicksPerMinute = (powerShotSpeed * 28) / 60;

    private double i_error = 0;
    private double d_error = 0;
    private double last_error = 0;
    private double last_time = 0;
    private double sample_time_millis = 0;



    public DriveTrain drive;

    public boolean breakOn = false;

    public static position robotPose = new position(0,0,0);





    public PATH_FOLLOWER_STATES path_follow_state = PATH_FOLLOWER_STATES.READY;
    public int PATH_INDEX = 0;
    public RobotClass() {

    }

    /**
     * init robot hardware
     * @param hwmap
     */
    public void init(HardwareMap hwmap) {
        FrontLeft = hwmap.get(DcMotor.class, "FrontLeft");
        FrontRight = hwmap.get(DcMotor.class, "FrontRight");
        BackLeft = hwmap.get(DcMotor.class, "BackLeft");
        BackRight = hwmap.get(DcMotor.class, "BackRight");
        lift = hwmap.get(DcMotor.class, "arm");
        intake = hwmap.get(DcMotor.class, "intake");
        shooter = hwmap.get(DcMotorEx.class,"shooter");
        shooterArm = hwmap.get(Servo.class, "shooterArm");
        shooterArm.setPosition(SHOOTER_ARM_IN);
        wrist = hwmap.get(Servo.class, "wrist");
        claw = hwmap.get(Servo.class, "claw");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new DriveTrain(FrontLeft,FrontRight,BackLeft,BackRight);



        if (breakOn) {
            setBreak();
        } else {
            setFloat();
        }

    }



    /**
     * set robot to use holding torque when power is 0
     */
    public void setBreak() {
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        breakOn = true;
    }



    /**
     * set to not use holding torque when power is 0 and just 'float'
     */
    public void setFloat() {
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        breakOn = false;
    }


    /**
     * drives the wobble goal arm to setpoint when called, must be called in a loop
     * @param setpoint
     */
    public void driveArmToSetpoint(double setpoint) {
        double current_time = System.currentTimeMillis();
        sample_time_millis = current_time - last_time;
        double position = lift.getCurrentPosition();

        double error = setpoint - position;
        double p_error = error;
        d_error = (error - last_error) / (sample_time_millis);
        i_error = i_error + (error * (sample_time_millis));
        if ((error >= 0 && last_error <= 0) || (error <= 0 && last_error >= 0)) {
            i_error = 0;
        }
        double output = (p_error * liftKp) + (i_error * liftKi) + (d_error * liftKd);
        last_error = error;

        lift.setPower(Range.clip(output,-0.5,0.5));


        last_time = current_time;

    }

    /**
     * set the wrist and claw of the wobble goal arm to the position to hold the wobble goal before the round start
     */
    public void setWristAndClawForAutoINIT() {
        wrist.setPosition(WRIST_FOR_AUTO_INIT);
        claw.setPosition(CLAW_CLOSED);
    }



    /**
     *
     * @return heading of t265 camera
     */
    public double getAngle() {
        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
        return robotPose.angle.getRadians();
    }

    /**
     * get the angle from roadrunner in the same form as that from the imu
     * @return corrected angle in radians
     */
    public double getAngleProper() {
        double rr_angle;
        if (robotPose.getAngleDegrees() >= 180 && robotPose.getAngleDegrees() <= 360) {
            rr_angle = 360 - robotPose.getAngleDegrees();
        } else {
            rr_angle = -robotPose.getAngleDegrees();
        }
        return Math.toRadians(rr_angle);
    }

    public double normalizeAngleRR(double angle) {
        double modifiedAngle = angle % TAU;
        modifiedAngle = (modifiedAngle + TAU) % TAU;
        return modifiedAngle;
    }



    /**
     * real gamer field relative driving
     * @param ySpeed
     * @param xSpeed
     * @param turnSpeed
     */
    public void FieldRelative(double ySpeed, double xSpeed, double turnSpeed) {
        double angle = Math.toDegrees(getAngleProper());
        xSpeed = clipMotor(xSpeed);
        ySpeed = clipMotor(ySpeed);
        turnSpeed = clipMotor(turnSpeed);

        Vector2d input = new Vector2d(xSpeed,ySpeed);

        input = input.rotateBy(angle);

        double theta = input.angle();

        frontLeftPower = input.magnitude() * Math.sin(theta + Math.PI / 4) + turnSpeed;
        frontRightPower = input.magnitude() * Math.sin(theta - Math.PI / 4) - turnSpeed;
        backLeftPower = input.magnitude() * Math.sin(theta - Math.PI / 4) + turnSpeed;
        backRightPower = input.magnitude() * Math.sin(theta + Math.PI / 4) - turnSpeed;

        drive.setMotorPowers(frontLeftPower,frontRightPower,backLeftPower,backRightPower);
    }


    /**
     * boring mecanum robot relative driving
     * @param xSpeed
     * @param ySpeed
     * @param turnSpeed
     */
    public void robotRelative(double xSpeed, double ySpeed, double turnSpeed) {
        frontLeftPower = xSpeed + ySpeed + turnSpeed;
        backLeftPower = xSpeed - ySpeed + turnSpeed;
        frontRightPower = xSpeed - ySpeed - turnSpeed;
        backRightPower = xSpeed + ySpeed - turnSpeed;
        drive.setMotorPowers(frontLeftPower,frontRightPower,backLeftPower,backRightPower);
    }


    /**
     * offsets a position by a particular starting pose to treat start as (0,0)
     * @param target the target position we are adjusting
     * @param start the Pose we want to be the relative 0,0
     * @return the adjusted target positon
     */
    public position adjustedTargetPosition(position target, Pose2d start) {
        target.setX(target.getX() + start.getTranslation().getX());
        target.setY(target.getY() + start.getTranslation().getY());

        return target;
    }


    /**
     * batch job adjusting of position
     * @param positions arraylist of positions that need adjusting
     * @param start the pose we consider the start
     * @return the arraylist of adjusted positions
     */
    public ArrayList<position> adjustedTargetPositionBatch(ArrayList<position> positions, Pose2d start) {
        ArrayList<position> adjustedTargetList = new ArrayList<>();
        for (int i = 0; i < positions.size(); i++) {
            adjustedTargetList.add(adjustedTargetPosition(positions.get(i),start));
        }
        return adjustedTargetList;

    }


    /**
     * the function field relative drive to point method
     * @param targetPose the current target the robot is driving towards
     */
    public void goodDriveToPoint(position targetPose) {
        double currentTime = (double) System.currentTimeMillis() / 1000;
        double kp = 14.693 * 0.01; // TODO: run printPosition with a 14v battery and get the fastest case transfer function
        double kd = 0.12627 * 0.05;
        double kpTurn = 0.75;
        double kdTurn = 0;

        double xError = targetPose.getX() - robotPose.getX();
        double yError = targetPose.getY() - robotPose.getY();

        double angle = getAngle();
        double targetAngle = targetPose.getAngleRadians();
        // We are epic so we assume that if the target angle is close to 180 and we are somewhat close to 180 we are at the target angle because we dont fw angle wrap
        headingError = AngleWrap(-normalizeAngleRR(targetAngle - angle));


        double d_error_x = (xError - last_error_x) / (currentTime - timeOfLastupdate);
        double d_error_y = (yError - last_error_y) / (currentTime - timeOfLastupdate);
        double d_error_heading = (headingError - last_error_angle) / (currentTime - timeOfLastupdate);

        xPower = (xError * kp) + (d_error_x * kd);
        yPower = (yError * kp) + (d_error_y * kd);
        yPower = -yPower; 
        turnPower = (headingError * kpTurn) + (d_error_heading * kdTurn);

        FieldRelative(xPower,yPower,turnPower);

        timeOfLastupdate = currentTime;
        last_error_x = xError;
        last_error_y = yError;
        last_error_angle = headingError;
    }

    /**
     * drives to position only
     * @param targetPose
     */
    public void driveToPointONLY(position targetPose) {
        double currentTime = (double) System.currentTimeMillis() / 1000;
        double kp = 14.693 * 0.01; // TODO: run printPosition with a 14v battery and get the fastest case transfer function
        double kd = 0.12627 * 0.01;
        double kpTurn = 0.55;
        double kdTurn = 0.01;

        double xError = targetPose.getX() - robotPose.getX();
        double yError = targetPose.getY() - robotPose.getY();


        // if the distance to the point is greater than the threshold, face in the points direction
        // if the robot is closer then face the target angle
        headingError = AngleWrap(Math.atan2(targetPose.getY() - robotPose.getY(),targetPose.getX() - robotPose.getX()) - getAngleProper());

        double d_error_x = (xError - last_error_x) / (currentTime - timeOfLastupdate);
        double d_error_y = (yError - last_error_y) / (currentTime - timeOfLastupdate);
        double d_error_heading = (headingError - last_error_angle) / (currentTime - timeOfLastupdate);

        xPower = (xError * kp) + (d_error_x * kd);
        yPower = (yError * kp) + (d_error_y * kd);
        yPower = -yPower;
        if (targetPose.distanceToPose(robotPose) < 7) {
            turnPower = (headingError * kpTurn) + (d_error_heading * kdTurn);
        }

        FieldRelative(xPower,yPower,turnPower);

        timeOfLastupdate = currentTime;
        last_error_x = xError;
        last_error_y = yError;
        last_error_angle = headingError;
    }

    /**
     * the function field relative drive to point method
     * @param targetPose the current target the robot is driving towards
     */
    public void goodDriveToPointWithMaxSpeed(position targetPose,double max_speed) {
        double currentTime = (double) System.currentTimeMillis() / 1000;
        double kp = 14.693 * 0.01; // TODO: run printPosition with a 14v battery and get the fastest case transfer function
        double kd = 0.12627 * 0.05;
        double kpTurn = 0.75;
        double kdTurn = 0;

        double xError = targetPose.getX() - robotPose.getX();
        double yError = targetPose.getY() - robotPose.getY();

        double angle = getAngle();
        double targetAngle = targetPose.getAngleRadians();
        // We are epic so we assume that if the target angle is close to 180 and we are somewhat close to 180 we are at the target angle because we dont fw angle wrap
        headingError = AngleWrap(-normalizeAngleRR(targetAngle - angle));


        double d_error_x = (xError - last_error_x) / (currentTime - timeOfLastupdate);
        double d_error_y = (yError - last_error_y) / (currentTime - timeOfLastupdate);
        double d_error_heading = (headingError - last_error_angle) / (currentTime - timeOfLastupdate);

        xPower = (xError * kp) + (d_error_x * kd);
        yPower = (yError * kp) + (d_error_y * kd);
        yPower = -yPower;
        turnPower = (headingError * kpTurn) + (d_error_heading * kdTurn);


        FieldRelative(Range.clip(xPower,-max_speed,max_speed),Range.clip(yPower,-max_speed,max_speed),Range.clip(turnPower,-max_speed,max_speed));

        timeOfLastupdate = currentTime;
        last_error_x = xError;
        last_error_y = yError;
    }
    /**
     * same good drive to point method but without aiming towards the point
     * and instead just always aiming toward the final angle
     * @param targetPose
     */
    public void goodDriveToPointNoAtan2(position targetPose) {
        double currentTime = (double) System.currentTimeMillis() / 1000;
        double kp = 14.693 * 0.01; // TODO: run printPosition with a 14v battery and get the fastest case transfer function
        double kd = 0.12627 * 0.01;
        double kpTurn = 0.6;

        double xError = targetPose.getX() - robotPose.getX();
        double yError = targetPose.getY() - robotPose.getY();



        headingError = AngleWrap(targetPose.getAngleRadians() - getAngleProper());

        double d_error_x = (xError - last_error_x) / (currentTime - timeOfLastupdate);
        double d_error_y = (yError - last_error_y) / (currentTime - timeOfLastupdate);


        xPower = (xError * kp) + (d_error_x * kd);
        yPower = (yError * kp) + (d_error_y * kd);
        yPower = -yPower;
        turnPower = headingError * kpTurn;

        FieldRelative(xPower,yPower,turnPower);

        timeOfLastupdate = currentTime;
        last_error_x = xError;
        last_error_y = yError;
    }
    /**
     * turn to angle associated with position not necessarily toward the position itself
     * @param positionOfAngle
     */
    public void turnToAngle(position positionOfAngle) {
        double kpTurn = 1;

        double targetAngle = positionOfAngle.getAngleRadians();
        headingError = AngleWrap(targetAngle - getAngleProper());

        robotRelative(0,0,headingError * kpTurn);

    }

    /**
     * finite state machine to follow a path of many position points.
     * @param positions arraylist of position
     * @return the current state of the path following from the PATH_FOLLOWER_STATES enum
     */
    public PATH_FOLLOWER_STATES followPath(ArrayList<position> positions) {

        position CURRENT_TARGET = positions.get(PATH_INDEX);
        double next_point_threshold = 5;
        switch (path_follow_state) {
            case READY:
                path_follow_state = PATH_FOLLOWER_STATES.MOVING_TO_POINT;
                break;

            case STOPPED:
                drive.setMotorPowers(0,0,0,0);
                break;
            case NEXT_POINT:
                // if we are not on the last point, we can switch to the next one
                if (PATH_INDEX != positions.size() - 1) {
                    PATH_INDEX += 1;
                    path_follow_state = PATH_FOLLOWER_STATES.MOVING_TO_POINT;
                } else {
                    path_follow_state = PATH_FOLLOWER_STATES.STOPPED;
                }
                break;
            case MOVING_TO_POINT:
                goodDriveToPoint(CURRENT_TARGET);
                if (robotPose.distanceToPose(CURRENT_TARGET) < next_point_threshold) {
                    path_follow_state = PATH_FOLLOWER_STATES.NEXT_POINT;
                }
                break;
            default:
                break;
        }

        //return robotPose.distanceToPose(CURRENT_TARGET);
        return path_follow_state;

    }


    /**
     * squish motor power between 0 and 1
     * @param power un squished motor power
     * @return squished
     */
    public double clipMotor(double power) {
        return Range.clip(power, -1,1);
    }

    public enum PATH_FOLLOWER_STATES {
        STOPPED,
        NEXT_POINT,
        MOVING_TO_POINT,
        START,
        READY
    }


}
