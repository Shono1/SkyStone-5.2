package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.util.Collections;

public class PID_Controller {
    int kP, kI, kD;
    double[] memory = new double[100];
    double[] I_memory = new double[100];
    double SP = 0;

    PID_Controller(int p, int i, int d, double sp){
        kP = p;
        kI = i;
        kD = d;
        SP = sp;
    }

    public void set_memory(double[] mem){
        memory = mem;
    }

    public void add_timestep(double step){
        memory[99] = step;
        Collections.rotate(Arrays.asList(memory), 1);
    }

    public void set_SP(double sp){
        SP = sp;
    }

    public double calculate_correction(){

        return 0.5;
    }

    private double[] get_PID(){
        double[] ret = {get_P(), get_I(), get_D()};
    }

    private double get_P(){
        return memory[0] - SP;
    }

    private double get_I(){

    }

    private double get_D(){
        
    }
}
