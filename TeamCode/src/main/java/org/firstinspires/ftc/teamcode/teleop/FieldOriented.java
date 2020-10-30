package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

@Deprecated
public class FieldOriented extends LinearOpMode {

    private RobotClass robot;
    private SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass();
        robot.init(hardwareMap);
        //drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData("ready to start","Press play!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.FieldRelative(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            telemetry.addData("robot angle",robot.getAngleIMU());
            telemetry.addData("robot degrees",Math.toDegrees(robot.getAngleIMU()));
            telemetry.update();
        }

    }
}
