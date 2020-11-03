package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.openftc.easyopencv.OpenCvCamera;

public class auto extends LinearOpMode {
    public RobotClass robot = new RobotClass();

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {

    }
    public void initialize() {
        robot.init(hardwareMap);
        robot.setWristAndClawForAutoINIT();
        telemetry.addData("ready to start!","press play!");
        telemetry.update();
    }

    public enum auto_states {
        START,
        DRIVE_TO_AVOID_STARTING_STACK,
        DRIVE_TO_POWER_SHOT,
        SHOOT_FIRST_POWER_SHOT,
        SHOOT_SECOND_POWER_SHOT,
        SHOOT_THIRD_POWER_SHOT,
        DRIVE_TO_WOBBLE_ZONE,
        PLACE_WOBBLE_GOAL,
        GO_TO_SECOND_WOBBLE_GOAL,
        GRAB_SECOND_WOBBLE_GOAL,
        DRIVE_TO_WOBBLE_ZONE_AGAIN,
        PLACE_WOBBLE_GOAL_2,
        to_second_wobble_spot,
        PARK_ON_LINE
    }
}
