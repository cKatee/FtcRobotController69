package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


public class DriveTrain {


    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;

    private final double TICKS_PER_ROTATION = 537.6;
    private double WHEEL_DIAMETER_CM = 50;
    private double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_CM / 1000;
    private final double COUNTS_PER_METER = (TICKS_PER_ROTATION) / (WHEEL_DIAMETER_CM * Math.PI);


    // used to calculate robot Velocity
    private double FrontLeft_velo = 0;
    private double FrontRight_velo = 0;
    private double BackLeft_velo = 0;
    private double BackRight_velo = 0;

    private double FrontLeft_CurrentEncoderTicks = 0;
    private double FrontRight_CurrentEncoderTicks = 0;
    private double BackLeft_CurrentEncoderTicks = 0;
    private double BackRight_CurrentEncoderTicks = 0;

    private double FrontLeft_LastEncoderTicks = 0;
    private double FrontRight_LastEncoderTicks = 0;
    private double BackLeft_LastEncoderTicks = 0;
    private double BackRight_LastEncoderTicks = 0;
    private long timeOfLastSpeedCalculation = 0;

    public final double TRACK_WIDTH = 12.5;

    public double last_FrontLeft_Power = 0;
    public double last_FrontRight_Power = 0;
    public double last_BackRight_Power = 0;
    public double last_BackLeft_Power = 0;

    public DriveTrain(DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight) {

        this.FrontLeft = FrontLeft;
        this.FrontRight = FrontRight;
        this.BackLeft = BackLeft;
        this.BackRight = BackRight;
    }


    /**
     * lynx optimized motor power setter
     * @param fl
     * @param fr
     * @param bl
     * @param br
     */
    public void setMotorPowers(double fl, double fr, double bl, double br) {

        fl = Range.clip(fl,-1,1);
        fr = Range.clip(fr, -1, 1);
        bl = Range.clip(bl, -1,1);
        br = Range.clip(br, -1, 1);


        if (last_FrontLeft_Power != fl) {
            FrontLeft.setPower(fl);
        }
        if (last_FrontRight_Power != fr) {
            FrontRight.setPower(fr);
        }
        if (last_BackLeft_Power != bl) {
            BackLeft.setPower(bl);
        }

        if (last_BackRight_Power != br) {
            BackRight.setPower(br);
        }

        last_BackLeft_Power = bl;
        last_BackRight_Power = br;
        last_FrontLeft_Power = fl;
        last_FrontRight_Power = fr;

    }

    public void STOP() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }


    /**
     * returns the number of meters for a given number of encoder counts
     * @param counts
     * @return
     */
    public double EncoderCountsToMeters(double counts) {
        return counts / COUNTS_PER_METER;
    }


}
