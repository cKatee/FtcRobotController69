package org.firstinspires.ftc.teamcode.controllers;

public class ILQRcontroller {

    // the plant state
    private double state = 0;
    // the controller output
    private double output = 0;
    // error penalizer
    private double Q = 0;
    private double R = 0;
    private double K = 0;


    /**
     *
     * @param Q
     * @param R
     * @param inital_state
     */
    public ILQRcontroller(double Q, double R, double inital_state) {

        this.Q = Q;
        this.R = R;
        this.state = inital_state;

    }

    /**
     * generate the output of the controller
     * @param state the plant error
     * @return
     */
    public double output(double state) {
        this.state = state;

        K = generateCurrentCost();
        K = this.state * K;

        return output;
    }

    /**
     * calculates the error and feeds that into the controller
     * @param setpoint
     * @param current_state
     * @return
     */
    public double output(double setpoint, double current_state) {
        return output(setpoint - current_state);
    }

    /**
     * generates the current quadratic cost of the plants state and the actuator input given weights Q and R
     * @returns the current quadratic cost
     */
    private double generateCurrentCost() {
        double cost = Q * state + R * output;
        double quadratic_cost = Math.pow(cost,2);
        return quadratic_cost;
    }

    

}
