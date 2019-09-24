package org.firstinspires.ftc.teamcode;

public class Field {
    private final int X_LENGTH = 365;
    private final int Y_LENGTH = 365;
    private int x_pos;
    private int y_pos;

    Field(int x, int y){
        x_pos = x;
        y_pos = y;
    }

    public void add_vec(int x, int y){
        x_pos += x;
        y_pos += y;
    }

    public int[] get_location(){
        int[] ret = {x_pos, y_pos};
        return ret;
    }

}
