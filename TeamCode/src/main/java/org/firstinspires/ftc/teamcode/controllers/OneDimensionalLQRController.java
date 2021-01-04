package org.firstinspires.ftc.teamcode.controllers;

public class OneDimensionalLQRController {

    // reference signal scaling term
    protected double Kr = 0;

    // gain 'matrix' K
    protected double K = 0;

    // output gain scaler
    protected double outputGain = 1;

    /**
     * constructor pogg
     * @param K gain K
     * @param Kr signal scaling
     */
    public OneDimensionalLQRController(double K, double Kr) {
        this.Kr = Kr;
        this.K = K;
        this.outputGain = 1;
    }

    /**
     * constructor with output scaling pogg
     * @param K gain K
     * @param Kr signal scaling
     * @param outputGain
     */
    public OneDimensionalLQRController(double K, double Kr, double outputGain) {
        this.K = K;
        this.Kr = Kr;
        this.outputGain = outputGain;
    }

    /**
     * outputs the signal from the LQR controller
     * @param reference the state we want to achieve
     * @param state the state we are at
     * @return the command to send to the motor
     */
    public double outputLQR(double reference, double state) {
        double scaledReference = reference * Kr;
        double scaledState = state * K;
        double error = scaledReference - scaledState;
        return error * outputGain;
    }



    public double getK() {
        return K;
    }

    public double getKr() {
        return Kr;
    }

    public double getOutputGain() {
        return outputGain;
    }


    public void setK(double k) {
        K = k;
    }

    public void setKr(double kr) {
        Kr = kr;
    }

    public void setOutputGain(double outputGain) {
        this.outputGain = outputGain;
    }


}
