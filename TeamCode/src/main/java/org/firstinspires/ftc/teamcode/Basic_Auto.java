package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Base class for all autonomous classes
 * Contains various methods for movement and sensing
 */
public class Basic_Auto extends LinearOpMode {
    protected static BNO055IMU imu;
    protected DcMotor fl;
    protected DcMotor bl;
    protected DcMotor fr;
    protected DcMotor br;

    protected DcMotor wr;
    protected DcMotor wl;

    private Servo fgl;
    private Servo fgr;
    private Servo bgl;
    private Servo bgr;

    protected PID_Controller x_pid, y_pid, heading_pid;
    protected double sp_angle = 0;

    protected ColorSensor clr_l;
    protected ColorSensor clr_r;

    protected DistanceSensor dist_r;
    protected DistanceSensor dist_l;

    private final double FLOOR = 0.4;

    /**
     * This is the code that is executed when the program is run
     * This just sets up all the hardware and prepositions motors to fit within the sizing cubs
     * @throws InterruptedException
     */
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

        dist_r = hardwareMap.get(DistanceSensor.class, "dist_r");
        dist_l = hardwareMap.get(DistanceSensor.class, "dist_l");

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
        heading_pid = new PID_Controller(1.7, 0.8, 0, 0);
        x_pid = new PID_Controller(0.00006, 0.0001, 0, 1);
        y_pid = new PID_Controller(0.00006, 0.0001, 0, 1);

        // Servo prepositioning
        bgl.setPosition(0);
        bgr.setPosition(0);
        // waitForStart();
    }

    /**
     * Old method for controlling 2 DoF arm
     * @deprecated
     * @param inner
     * @param outer
     */
    private void setArmPosition(double inner, double outer){
        fgl.setPosition(inner);
        bgl.setPosition(outer);
        fgr.setPosition(1 - inner);
        bgr.setPosition(1 - outer);
    }

    /**
     * Method to move to polar coordinate where the robot is the origin
     * It is reccomended to use floor move instead of this method
     * @param r
     * @param theta
     */
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

    /**
     * Rotates the robot; does not play well with other methods;
     * Don't use
     * @param radians
     */
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

    /**
     * Set gripper servos to position to grab block
     */
    protected void grab(){
        fgl.setPosition(0);
        fgr.setPosition(1);
        sleep(1500);

    }

    /**
     * Set gripper to position to release blocks
     */
    protected void release(){
        fgl.setPosition(1);
        fgr.setPosition(0);
        sleep(700);
    }

    /**
     * @Deprecated
     * USed to be used to move foundation
     * Replaced with floor move
     */
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

    /**
     * Checks if robot is in front of a skystone
     * @return
     */
    protected boolean sense_block(){
        if(is_black(clr_r) && is_black(clr_l)) return true;
        return false;
    }

    /**
     * Checks if a color sensor is within the range adequate to be called black
     * @param clr
     * @return
     */
    protected boolean is_black(ColorSensor clr){
        if(clr.red() < 65 && clr.blue() < 65 && clr.green() < 65) return true;
        return false;
    }

    /**
     * Used to control 2 DoF arm
     * Clamps 2 DoF arm down on what should be a skystone
     */
    protected void grab_block(){
        fgl.setPosition(0.4);
        fgr.setPosition(0.5);
        bgl.setPosition(1);
        bgr.setPosition(1);
    }

    /**
     * Method used to move along wall of blocks and stop when one of them is black
     * Stays constant 2cm away from wall
     */
    protected void move_and_sense(){
        // How far it needs to go
        int x_steps = -45_000;
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
                (heading_pid.get_PV() > heading_pid.get_SP() + 0.02 || heading_pid.get_PV() <
                        heading_pid.get_SP() - 0.02)) && opModeIsActive()){
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

    /**
     * Old foundation moving method, use floor move
     * @Deprecated
     */
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

    /**
     * Old foundation moving method, use floor move
     * @Deprecated
     */
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

    /**
     * Currently the standard movement method. Uses sprung odometry wheels to measure position,
     * and uses PID controllers to control x and y components of movement. Has a floor that power
     * can not go under if value is not in optimal range.
     * @param r
     * @param theta
     */
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

    /**
     * Special case method for FloorMove
     * @param r
     * @param theta
     */
    public void specialFloorMove(int r, double theta){
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
            fl.setPower(ratio * Range.clip((-y - x + rotation), -1, 1));
            fr.setPower(ratio * Range.clip((y - x + rotation), -1, 1));
            br.setPower(ratio * Range.clip((y + x + rotation), -1, 1));
            bl.setPower(ratio * Range.clip((-y + x + rotation), -1, 1));

            // Set PID setpoints
            int x_pos, y_pos;

            x_pos = wl.getCurrentPosition();
            y_pos = wr.getCurrentPosition();

            x_pid.set_PV(x_pos); // x port 0
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

    /**
     * Floor move where you can set your own floor
     * @param r
     * @param theta
     * @param floor
     */
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

    /**
     * Method to get angle between two points
     * @deprecated
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    private double get_angle(int x1, int y1, int x2, int y2){
        int x_delta = x1 - x2;
        int y_delta = y1 - y2;

        double angle = Math.atan2((double) y_delta, x_delta);

        return angle;
    }

    /**
     * Method to get angle between two points
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return
     */
    private double get_angle(double x1, double y1, double x2, double y2){
        double x_delta = x1 - x2;
        double y_delta = y1 - y2;

        double angle = Math.atan2(y_delta, x_delta);

        return angle;
    }

    /**
     * Method to move until 2 cm away from wall
     * @param d
     */
    protected void moveToBlocks(double d){

        // TODO: write code to make bot move with distance sensors
        PID_Controller dl_pid = new PID_Controller(0.0925, 0, 0, d);
        PID_Controller dr_pid = new PID_Controller(0.0925, 0, 0, d);
        heading_pid.set_SP(0);
        // PID_Controller x_pid = new PID_Controller(1.5, 0.8, 0, 0);

        double floor = 0.3;
        int count = 0;
        int max_count = 45;

        while(((dl_pid.get_PV() > dl_pid.get_SP() + 1.5 || dl_pid.get_PV() < dl_pid.get_SP() - 1.5) ||
                (dr_pid.get_PV() > dr_pid.get_SP() + 1.5 || dr_pid.get_PV() < dr_pid.get_SP() - 1.5))
                        && opModeIsActive())
        {
            dl_pid.set_PV(dist_l.getDistance(DistanceUnit.CM));
            dr_pid.set_PV(dist_r.getDistance(DistanceUnit.CM));
            // Get movement components
            count++;
            double ratio = (double) count / max_count;
            double r, y, rotation;


            rotation = heading_pid.get_PID();


            // Set motor powers
            fl.setPower(0.6 * Range.clip(ratio * -dl_pid.get_PID(), -1, 1));
            fr.setPower(0.6 * Range.clip(ratio * dr_pid.get_PID(), -1, 1));
            br.setPower(0.6 * Range.clip(ratio * dr_pid.get_PID(), -1, 1));
            bl.setPower(0.6 * Range.clip(ratio * -dl_pid.get_PID(), -1, 1));

            telemetry.addData("dist l", dist_l.getDistance(DistanceUnit.CM));
            telemetry.addData("dist r", dist_r.getDistance(DistanceUnit.CM));
            telemetry.addData("dl_pid", dl_pid.get_PID());
            telemetry.addData("dr_pid", dr_pid.get_PID());
            telemetry.addData("heading correction", heading_pid.get_PID());
            telemetry.addData("temp", Range.clip((ratio * -dr_pid.get_PID()), -1, 1));
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    /**
     * Move along blocks with a red or blue side flag
     * @param r
     * @param d
     * @param side
     */
    protected void moveAlongBlocks(int r, double d, boolean side){
        PID_Controller dl_pid = new PID_Controller(0.0925, 0, 0, d);
        PID_Controller dr_pid = new PID_Controller(0.0925, 0, 0, d);
        PID_Controller r_pid = new PID_Controller(0.00006, 0.0001, 0, r);
        heading_pid.set_SP(0);
        // PID_Controller x_pid = new PID_Controller(1.5, 0.8, 0, 0);

        double floor = 0.3;
        int count = 0;
        int max_count = 45;
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(((dl_pid.get_PV() > dl_pid.get_SP() + 1.5 || dl_pid.get_PV() < dl_pid.get_SP() - 1.5) ||
                (dr_pid.get_PV() > dr_pid.get_SP() + 1.5 || dr_pid.get_PV() < dr_pid.get_SP() - 1.5) ||
                r_pid.get_PV() > r_pid.get_SP() + 1000 || r_pid.get_PV() < r_pid.get_SP() - 1000)
                && opModeIsActive())
        {
            dl_pid.set_PV(dist_l.getDistance(DistanceUnit.CM));
            dr_pid.set_PV(dist_r.getDistance(DistanceUnit.CM));
            // Get movement components
            count++;
            double ratio = (double) count / max_count;
            double x, y, rotation;

            r_pid.set_PV(wl.getCurrentPosition());
            x = r_pid.get_PID();
            if(x < floor && r_pid.get_PV() > r_pid.get_SP() + 1000 || r_pid.get_PV() < r_pid.get_SP() - 1000)
                x = floor;

            if(sense_block()) break;

            if(side) {
                // Set motor powers
                fl.setPower(0.6 * Range.clip(ratio * -dl_pid.get_PID() + x, -1, 1));
                fr.setPower(0.6 * Range.clip(ratio * dr_pid.get_PID() + x, -1, 1));
                br.setPower(0.6 * Range.clip(ratio * dr_pid.get_PID() - x, -1, 1));
                bl.setPower(0.6 * Range.clip(ratio * -dl_pid.get_PID() - x, -1, 1));
            }
            else{
                fl.setPower(0.6 * Range.clip(ratio * -dl_pid.get_PID() - x, -1, 1));
                fr.setPower(0.6 * Range.clip(ratio * dr_pid.get_PID() - x, -1, 1));
                br.setPower(0.6 * Range.clip(ratio * dr_pid.get_PID() + x, -1, 1));
                bl.setPower(0.6 * Range.clip(ratio * -dl_pid.get_PID() + x, -1, 1));
            }

            telemetry.addData("dist l", dist_l.getDistance(DistanceUnit.CM));
            telemetry.addData("dist r", dist_r.getDistance(DistanceUnit.CM));
            telemetry.addData("dl_pid", dl_pid.get_PID());
            telemetry.addData("dr_pid", dr_pid.get_PID());
            telemetry.addData("x ticks", wl.getCurrentPosition());
            telemetry.addData("x pid", r_pid.get_PID());
            telemetry.addData("heading correction", heading_pid.get_PID());
            telemetry.addData("temp", Range.clip((ratio * -dr_pid.get_PID()), -1, 1));
            telemetry.addData("clr left", is_black(clr_l) + " " + clr_l.toString());
            telemetry.addData("clr right", is_black(clr_r) + " " + clr_r.toString());
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }


}
