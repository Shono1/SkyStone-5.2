package org.firstinspires.ftc.teamcode;

public class PID_Controller {
    private double kP, kI, kD;
    //private double[] PV_memory = new double[2];
    private double I_sum = 0;
    private double I_count = 0;
    //private double[] SP_memory = new double[2];
    private double SP, last_SP;
    private double PV, last_PV;
    private double dt = 0.001;
    private double I = 0;
    private double P, last_P = 0;

    /**
     * Proportional Integral Derivative Controller
     * Controls a process variable in relation to a setpoint
     * For best results, tune heavily for each purpose
     * @param p
     * @param i
     * @param d
     * @param sp
     */
    PID_Controller(double p, double i, double d, double sp){
        kP = p;
        kI = i;
        kD = d;
        SP = sp;
    }

    public void set_SP(double sp){
        last_SP = SP;
        SP = sp;
    }

    public double get_SP(){
        return SP;
    }

    public void set_PV(double pv){
        last_PV = PV;
        PV = pv;
    }

    public double calculate_correction(){

        return 0.5;
    }

    /**
     * Returns the result of the PID controller
     * @return
     */
    public double get_PID(){
        return kP * get_P() + kI * get_I() + kD * get_D();
    }

    private double get_P(){
        P = SP - PV;
        return SP - PV;
    }

    private double get_I(){
        I = I + get_P() * dt;
        return I + get_P() * dt;
    }

    private double get_D(){
        double ret = (P - last_P) / dt;
        last_P = P;
        return ret;
    }

    public double get_PV(){
        return PV;
    }
}
