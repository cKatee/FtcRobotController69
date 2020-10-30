package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotClass;

@Disabled
public class testMotors extends OpMode {
    private RobotClass robot = new RobotClass();

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("ready","");
    }

    @Override
    public void loop() {

        robot.drive.setMotorPowers(1,1,1,1);
        long time = System.currentTimeMillis();
        while (System.currentTimeMillis() < time + 1000)  {
            telemetry.addData("bl",robot.BackLeft.getCurrentPosition());
            telemetry.addData("br",robot.BackRight.getCurrentPosition());
            telemetry.addData("fl",robot.FrontLeft.getCurrentPosition());
            telemetry.addData("fr",robot.FrontRight.getCurrentPosition());
            telemetry.update();
        }
        robot.drive.STOP();
        time = System.currentTimeMillis();
        while (true)  {
            telemetry.addData("bl",robot.BackLeft.getCurrentPosition());
            telemetry.addData("br",robot.BackRight.getCurrentPosition());
            telemetry.addData("fl",robot.FrontLeft.getCurrentPosition());
            telemetry.addData("fr",robot.FrontRight.getCurrentPosition());
            telemetry.update();
        }

    }

}
