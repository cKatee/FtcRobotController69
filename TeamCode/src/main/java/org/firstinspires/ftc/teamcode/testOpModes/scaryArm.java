package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;

@Disabled
public class scaryArm extends LinearOpMode {
    private RobotClass robot;
    private double MAX_HEIGHT = 104;


    @Override
    public void runOpMode() {
        robot = new RobotClass();
        robot.init(hardwareMap);
        telemetry.addData("ready to start!","press play!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.driveArmToSetpoint(MAX_HEIGHT);
            telemetry.addData("speed",robot.lift.getPower());
            telemetry.update();
        }

    }

    private void waitForTime(long time) {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < startTime + time) {

        }
    }
}
