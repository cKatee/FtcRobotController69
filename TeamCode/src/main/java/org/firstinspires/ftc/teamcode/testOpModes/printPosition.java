package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@TeleOp
public class printPosition extends LinearOpMode {

    private RobotClass robot;
    private BNO055IMU imu;

    BNO055IMU.Parameters IMU_Parameters;

    @Override
    public void runOpMode() {

        //path = robot.adjustedTargetPositionBatch(path, new com.arcrobotics.ftclib.geometry.Pose2d(0,0,new Rotation2d(0)));

        SampleMecanumDrive roadrunnerOdometry = new SampleMecanumDrive(hardwareMap);
        robot = new RobotClass();
        robot.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        IMU_Parameters = new BNO055IMU.Parameters();

        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Create an IMU parameters object.
        IMU_Parameters = new BNO055IMU.Parameters();


        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU gyroscope and accelerometer to
        // calculate the relative orientation of hub and
        // therefore the robot.
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Intialize the IMU using parameters object.
        imu.initialize(IMU_Parameters);
        // Report the initialization to the Driver Station.


        // Loop until IMU has been calibrated.
        while (!IMU_Calibrated()) {

        }

        waitForStartReady();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {

            timer.reset();
            roadrunnerOdometry.updatePoseEstimate();

            Pose2d pose = roadrunnerOdometry.getPoseEstimate();
            Acceleration accel = imu.getLinearAcceleration();
            Orientation gyroOut = imu.getAngularOrientation();
            robot.robotPose.setPose2dRoadRunner(pose);
            robot.FieldRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            double currentTime = timer.milliseconds() / 1000;
            System.out.println("" + pose.getX() + ", " +pose.getY() + ", " + currentTime + ", " + accel.xAccel + ", " + accel.yAccel + ", " + gyroOut.firstAngle + ", " + gyroOut.secondAngle + ", " + gyroOut.thirdAngle);




        }


    }

    public void waitForStartReady() {
        telemetry.addData("Ready to Start!","Press play to begin!");
        telemetry.update();
        waitForStart();
    }
    private boolean IMU_Calibrated() {
        return imu.isGyroCalibrated();
    }
}
