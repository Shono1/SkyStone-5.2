package org.firstinspires.ftc.teamcode;

import java.util.Collections;

public class PID_Controller {
    private int kP, kI, kD;
    private double[] PV_memory = new double[100];
    private double I_sum = 0;
    private double I_count = 0;
    private double[] SP_memory = new double[100];
    private double SP;

    PID_Controller(int p, int i, int d, double sp){
        kP = p;
        kI = i;
        kD = d;
        SP = sp;
    }

    public void set_memory(double[] mem){
        PV_memory = mem;
    }

    public void add_timestep(double PV, double SP){
        PV_memory[99] = PV;
        Collections.rotate(Collections.singletonList(PV_memory), 1);
        SP_memory[99] = SP;
        Collections.rotate(Collections.singletonList(SP_memory), 1);
    }

    public void set_SP(double sp){
        SP = sp;
    }

    public double calculate_correction(){

        return 0.5;
    }

    private double[] get_PID(){
        return new double[]{kP * get_P(), kI * get_I(), kD * get_D()};
    }

    private double get_P(){
        return PV_memory[0] - SP;
    }

    private double get_I(){
        I_sum += PV_memory[1] - PV_memory[0];
        I_count++;
        return I_sum / I_count;
    }

    private double get_D(){
        return (PV_memory[1] - SP_memory[1]) - (PV_memory[0] - SP_memory[0]) / 1; //Mess around with "dt" a little bit here
    }
}
