package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotClass;

@Disabled
public class FrontLeftAsIntake extends LinearOpMode {
    private RobotClass robot;
    private DcMotor FrontLeft;
    @Override
    public void runOpMode() {
        FrontLeft = hardwareMap.get(DcMotor.class,"intake");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("ready to start!",true);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            FrontLeft.setPower(1);
        }
        FrontLeft.setPower(0);

    }
}
