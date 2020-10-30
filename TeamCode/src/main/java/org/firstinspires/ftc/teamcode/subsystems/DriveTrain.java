package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;


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

    public DriveTrain(DcMotor FrontLeft, DcMotor FrontRight, DcMotor BackLeft, DcMotor BackRight) {

        this.FrontLeft = FrontLeft;
        this.FrontRight = FrontRight;
        this.BackLeft = BackLeft;
        this.BackRight = BackRight;
    }


    public void setMotorPowers(double fl, double fr, double bl, double br) {
        FrontLeft.setPower(fl);
        FrontRight.setPower(fr);
        BackLeft.setPower(bl);
        BackRight.setPower(br);
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
