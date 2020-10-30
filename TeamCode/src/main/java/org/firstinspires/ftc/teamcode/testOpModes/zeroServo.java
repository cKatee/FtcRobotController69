package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class zeroServo extends LinearOpMode {

    private Servo armServo1;

    @Override
    public void runOpMode() throws InterruptedException {
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        armServo1.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {
            armServo1.setPosition(0.5);
        }
    }
}
