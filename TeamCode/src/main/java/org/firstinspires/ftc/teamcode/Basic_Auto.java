package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Basic_Auto extends LinearOpMode {
    protected BNO055IMU imu;
    protected DcMotor fl;
    protected DcMotor bl;
    protected DcMotor fr;
    protected DcMotor br;

    protected DcMotor wr;
    protected DcMotor wl;
    protected DcMotor dummy;

    private Servo lg;
    private Servo rg;

    private Servo fgl;
    private Servo fgr;
    private Servo bgl;
    private Servo bgr;

    protected PID_Controller x_pid, y_pid, heading_pid;
    protected double sp_angle = 0;

    protected ColorSensor clr_l;
    protected ColorSensor clr_r;

    private final double FLOOR = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        //Dc Motor Initialization
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        //Winch Initialization
        wr = hardwareMap.get(DcMotor.class, "wr");
        wl = hardwareMap.get(DcMotor.class, "wl");

        // Servo Arm Initialization
        fgl = hardwareMap.get(Servo.class, "l_found");
        fgr = hardwareMap.get(Servo.class, "r_found");
        bgl = hardwareMap.get(Servo.class, "ablock_1");
        bgr = hardwareMap.get(Servo.class, "ablock_2");

        clr_l = hardwareMap.get(ColorSensor.class, "clr_l");
        clr_r = hardwareMap.get(ColorSensor.class, "clr_r");

        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        VectorF controller_vector = new VectorF(0,0);
        VectorF robot_vector = new VectorF(0,0);
        float[] temp_orientation = {1, 0, 0, 1};
        GeneralMatrixF orientation = new GeneralMatrixF(2,2, temp_orientation);
        float robot_angle = 0;
        float[] temp = new float[4];




        // Odometer PID initialization
        heading_pid = new PID_Controller(1.5, 0.8, 0, 0);
        x_pid = new PID_Controller(0.00006, 0.0001, 0, 1);
        y_pid = new PID_Controller(0.00006, 0.0001, 0, 1);

        // Servo prepositioning
        bgl.setPosition(0);
        bgr.setPosition(0);
        // waitForStart();
    }

    private void setArmPosition(double inner, double outer){
        fgl.setPosition(inner);
        bgl.setPosition(outer);
        fgr.setPosition(1 - inner);
        bgr.setPosition(1 - outer);
    }

    protected void polarMove(int r, double theta){
        // How far it needs to go
        int x_steps = (int) (Math.cos(theta) * r);
        int y_steps = (int) (Math.sin(theta) * r);


        // Giving PIDs setpoints
        x_pid.set_SP(x_steps);
        x_pid.set_PV(0);

        y_pid.set_SP(y_steps);
        y_pid.set_PV(0);

        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        heading_pid.set_SP(sp_angle);
        heading_pid.set_PV(imu.getAngularOrientation().firstAngle);

        // Setting speed up parameters
        int count = 0;
        int max_count = 30;

        // Clear encoders

        // dummy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Moving robot
        while(((x_pid.get_PV() > x_pid.get_SP() + 1000 || x_pid.get_PV() < x_pid.get_SP() - 10000) ||
                (y_pid.get_PV() > y_pid.get_SP() + 1000 || y_pid.get_PV() < y_pid.get_SP() - 1000) ||
                (heading_pid.get_PV() > heading_pid.get_SP() + 0.05 || heading_pid.get_PV() <
                        heading_pid.get_SP() - 0.05)) && opModeIsActive()){
            // Gradual Speed Up
            if(count < max_count) count++;
            double ratio = (double) count / max_count;

            // Get movement components
            double x, y, rotation;
            x = x_pid.get_PID();
            y = y_pid.get_PID();
            rotation = heading_pid.get_PID();

            // Set motor powers
            fl.setPower(ratio * Range.clip((-y + x + rotation), -1, 1));
            fr.setPower(ratio * Range.clip((y + x + rotation), -1, 1));
            br.setPower(ratio * Range.clip((y - x + rotation), -1, 1));
            bl.setPower(ratio * Range.clip((-y - x + rotation), -1, 1));

            // Set PID setpoints
            int x_pos, y_pos;

            x_pos = wl.getCurrentPosition();
            y_pos = wr.getCurrentPosition();

            x_pid.set_PV(-x_pos); // x port 0
            y_pid.set_PV(y_pos); // y port 1
            heading_pid.set_PV(imu.getAngularOrientation().firstAngle); // imu

            telemetry.addData("x target", x_pid.get_SP());
            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("y target", y_pid.get_SP());
            telemetry.addData("y ticks", wr.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Turn off motors
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    protected void rotate(double radians){
        sp_angle += radians;
        heading_pid.set_SP(sp_angle);


        while(heading_pid.get_PV() > heading_pid.get_SP() - 0.05 ||
                heading_pid.get_PV() < heading_pid.get_SP() + 0.05 && opModeIsActive()){


            fl.setPower(heading_pid.get_PID());
            bl.setPower(heading_pid.get_PID());
            fr.setPower(heading_pid.get_PID());
            br.setPower(heading_pid.get_PID());

            idle();
        }

        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);

    }

    protected void grab(){
        fgl.setPosition(0);
        fgr.setPosition(1);
        sleep(1500);

    }

    protected void release(){
        fgl.setPosition(1);
        fgr.setPosition(0);
        sleep(700);
    }

    protected void move_foundation(){
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int count = 0;
        int max_count = 30;

        while(wr.getCurrentPosition() < 25000 && opModeIsActive()){
            if(count < max_count) count++;
            double ratio = (double) count / max_count;
            fl.setPower(ratio * -1);
            fr.setPower(ratio * 1);
            br.setPower(ratio * 1);
            bl.setPower(ratio * -1);

            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("y ticks", wr.getCurrentPosition());
            telemetry.update();
            idle();
        }

        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);

    }

    protected boolean sense_block(){
        if(is_black(clr_r) && is_black(clr_l)) return true;
        return false;
    }

    protected boolean is_black(ColorSensor clr){
        if(clr.red() < 10 && clr.blue() < 10 && clr.green() < 10) return true;
        return false;
    }

    protected void grab_block(){
        fgl.setPosition(0.25);
        fgr.setPosition(0.6);
        bgl.setPosition(1);
        bgr.setPosition(0.8);
    }

    protected void move_and_sense(){
        // How far it needs to go
        int x_steps = 45_000;
        int y_steps = 0;


        // Giving PIDs setpoints
        x_pid.set_SP(x_steps);
        x_pid.set_PV(0);

        y_pid.set_SP(y_steps);
        y_pid.set_PV(0);

        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        heading_pid.set_SP(sp_angle);
        heading_pid.set_PV(imu.getAngularOrientation().firstAngle);

        // Setting speed up parameters
        int count = 0;
        int max_count = 30;

        // Clear encoders

        // dummy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Moving robot
        while(((x_pid.get_PV() > x_pid.get_SP() + 1500 || x_pid.get_PV() < x_pid.get_SP() - 1500) ||
                (y_pid.get_PV() > y_pid.get_SP() + 1500 || y_pid.get_PV() < y_pid.get_SP() - 1500) ||
                (heading_pid.get_PV() > heading_pid.get_SP() + 0.05 || heading_pid.get_PV() <
                        heading_pid.get_SP() - 0.05)) && opModeIsActive()){
            // Gradual Speed Up
            if(count < max_count) count++;
            double ratio = (double) count / max_count;

            // Get movement components
            double x, y, rotation;
            x = x_pid.get_PID();
            y = y_pid.get_PID();
            rotation = heading_pid.get_PID();

            if(sense_block()){
                break;
            }

            // Set motor powers
            fl.setPower(0.5 * ratio * Range.clip((-y + x + rotation), -1, 1));
            fr.setPower(0.5 * ratio * Range.clip((y + x + rotation), -1, 1));
            br.setPower(0.5 * ratio * Range.clip((y - x + rotation), -1, 1));
            bl.setPower(0.5 * ratio * Range.clip((-y - x + rotation), -1, 1));

            // Set PID setpoints
            int x_pos, y_pos;

            x_pos = wl.getCurrentPosition();
            y_pos = wr.getCurrentPosition();

            x_pid.set_PV(-x_pos); // x port 0
            y_pid.set_PV(y_pos); // y port 1
            heading_pid.set_PV(imu.getAngularOrientation().firstAngle); // imu

            telemetry.addData("x target", x_pid.get_SP());
            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("y target", y_pid.get_SP());
            telemetry.addData("y ticks", wr.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Turn off motors
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void red_fnd_start(){
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int count = 0;
        int max_count = 30;

        while(wl.getCurrentPosition() > -4000 && opModeIsActive()){
            if(count < max_count) count++;
            double ratio = (double) count / max_count;
            fl.setPower(ratio * 1);
            fr.setPower(ratio * 1);
            br.setPower(ratio * -1);
            bl.setPower(ratio * -1);

            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("y ticks", wr.getCurrentPosition());
            telemetry.update();
            idle();
        }

        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void blue_fnd_start(){
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int count = 0;
        int max_count = 30;

        while(wl.getCurrentPosition() > -4000 && opModeIsActive()){
            if(count < max_count) count++;
            double ratio = (double) count / max_count;
            fl.setPower(ratio * 1);
            fr.setPower(ratio * 1);
            br.setPower(ratio * -1);
            bl.setPower(ratio * -1);

            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("y ticks", wr.getCurrentPosition());
            telemetry.update();
            idle();
        }

        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void floorMove(int r, double theta){
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // TODO: Add speed floor (lim pid -> 0 = floor(eg 0.4)
        // How far it needs to go
        int x_steps = (int) (Math.cos(theta) * r);
        int y_steps = (int) (Math.sin(theta) * r);


        // Giving PIDs setpoints
        x_pid.set_SP(x_steps);
        x_pid.set_PV(0);

        y_pid.set_SP(y_steps);
        y_pid.set_PV(0);

        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        heading_pid.set_SP(sp_angle);
        heading_pid.set_PV(imu.getAngularOrientation().firstAngle);

        // Setting speed up parameters
        int count = 0;
        int max_count = 30;

        // Clear encoders

        // dummy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Moving robot
        while(((x_pid.get_PV() > x_pid.get_SP() + 1000 || x_pid.get_PV() < x_pid.get_SP() - 1000) ||
                (y_pid.get_PV() > y_pid.get_SP() + 1000 || y_pid.get_PV() < y_pid.get_SP() - 1000) ||
                (heading_pid.get_PV() > heading_pid.get_SP() + 0.05 || heading_pid.get_PV() <
                        heading_pid.get_SP() - 0.05)) && opModeIsActive()){
            // Gradual Speed Up
            if(count < max_count) count++;
            double ratio = (double) count / max_count;

            // Get movement components
            double x, y, rotation;
            if(x_pid.get_PV() > x_pid.get_SP() + 1000 || x_pid.get_PV() < x_pid.get_SP() - 1000)
                x = x_pid.get_PID() + FLOOR * Math.cos(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV()));
            else x = 0;

            if(y_pid.get_PV() > y_pid.get_SP() + 1000 || y_pid.get_PV() < y_pid.get_SP() - 1000)
                y = y_pid.get_PID() + FLOOR * Math.sin(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV()));
            else y = 0;

            rotation = heading_pid.get_PID();

            // Set motor powers
            fl.setPower(ratio * Range.clip((-y + x + rotation), -1, 1));
            fr.setPower(ratio * Range.clip((y + x + rotation), -1, 1));
            br.setPower(ratio * Range.clip((y - x + rotation), -1, 1));
            bl.setPower(ratio * Range.clip((-y - x + rotation), -1, 1));

            // Set PID setpoints
            int x_pos, y_pos;

            x_pos = wl.getCurrentPosition();
            y_pos = wr.getCurrentPosition();

            x_pid.set_PV(-x_pos); // x port 0
            y_pid.set_PV(y_pos); // y port 1
            heading_pid.set_PV(imu.getAngularOrientation().firstAngle); // imu

            telemetry.addData("x target", x_pid.get_SP());
            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("y target", y_pid.get_SP());
            telemetry.addData("y ticks", wr.getCurrentPosition());
            telemetry.addData("angle between points",
                    get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV()));
            telemetry.addData("cos * floor", FLOOR * Math.cos(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV())));
            telemetry.addData("sin * floor", FLOOR * Math.sin(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV())));

            telemetry.update();
            idle();
        }

        // Turn off motors
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void floorMove(int r, double theta, double floor){
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // TODO: Add speed floor (lim pid -> 0 = floor(eg 0.4)
        // How far it needs to go
        int x_steps = (int) (Math.cos(theta) * r);
        int y_steps = (int) (Math.sin(theta) * r);


        // Giving PIDs setpoints
        x_pid.set_SP(x_steps);
        x_pid.set_PV(0);

        y_pid.set_SP(y_steps);
        y_pid.set_PV(0);

        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        heading_pid.set_SP(sp_angle);
        heading_pid.set_PV(imu.getAngularOrientation().firstAngle);

        // Setting speed up parameters
        int count = 0;
        int max_count = 30;

        // Clear encoders

        // dummy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Moving robot
        while(((x_pid.get_PV() > x_pid.get_SP() + 1000 || x_pid.get_PV() < x_pid.get_SP() - 1000) ||
                (y_pid.get_PV() > y_pid.get_SP() + 1000 || y_pid.get_PV() < y_pid.get_SP() - 1000) ||
                (heading_pid.get_PV() > heading_pid.get_SP() + 0.05 || heading_pid.get_PV() <
                        heading_pid.get_SP() - 0.05)) && opModeIsActive()){
            // Gradual Speed Up
            if(count < max_count) count++;
            double ratio = (double) count / max_count;

            // Get movement components
            double x, y, rotation;
            if(x_pid.get_PV() > x_pid.get_SP() + 1000 || x_pid.get_PV() < x_pid.get_SP() - 1000)
                x = x_pid.get_PID() + floor * Math.cos(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV()));
            else x = 0;

            if(y_pid.get_PV() > y_pid.get_SP() + 1000 || y_pid.get_PV() < y_pid.get_SP() - 1000)
                y = y_pid.get_PID() + floor * Math.sin(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV()));
            else y = 0;

            rotation = heading_pid.get_PID();

            // Set motor powers
            fl.setPower(ratio * Range.clip((-y + x + rotation), -1, 1));
            fr.setPower(ratio * Range.clip((y + x + rotation), -1, 1));
            br.setPower(ratio * Range.clip((y - x + rotation), -1, 1));
            bl.setPower(ratio * Range.clip((-y - x + rotation), -1, 1));

            // Set PID setpoints
            int x_pos, y_pos;

            x_pos = wl.getCurrentPosition();
            y_pos = wr.getCurrentPosition();

            x_pid.set_PV(-x_pos); // x port 0
            y_pid.set_PV(y_pos); // y port 1
            heading_pid.set_PV(imu.getAngularOrientation().firstAngle); // imu

            telemetry.addData("x target", x_pid.get_SP());
            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("y target", y_pid.get_SP());
            telemetry.addData("y ticks", wr.getCurrentPosition());
            telemetry.addData("angle between points",
                    get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV()));
            telemetry.addData("cos * floor", floor * Math.cos(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV())));
            telemetry.addData("sin * floor", floor * Math.sin(get_angle(x_pid.get_SP(), y_pid.get_SP(), x_pid.get_PV(), y_pid.get_PV())));

            telemetry.update();
            idle();
        }

        // Turn off motors
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    private double get_angle(int x1, int y1, int x2, int y2){
        int x_delta = x1 - x2;
        int y_delta = y1 - y2;

        double angle = Math.atan2((double) y_delta, x_delta);

        return angle;
    }

    private double get_angle(double x1, double y1, double x2, double y2){
        double x_delta = x1 - x2;
        double y_delta = y1 - y2;

        double angle = Math.atan2(y_delta, x_delta);

        return angle;
    }




}
