package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Field_Coordinate_Auto extends LinearOpMode {
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
    private int x = 0;
    private int y = 0;

    /**
     * Runs when opMode is initialized. Initializes all of the hardware and heading controllers
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

        // Odometer PID initialization
        heading_pid = new PID_Controller(1.7, 0.8, 0, 0);
        x_pid = new PID_Controller(0.00006, 0.0001, 0, 1);
        y_pid = new PID_Controller(0.00006, 0.0001, 0, 1);

        // Servo prepositioning
        bgl.setPosition(0);
        bgr.setPosition(0);
    }

    /**
     * Move the robot to a point on the field, holding heading
     * @param x
     * @param y
     */
    protected void move_to_point(int x, int y){
        int x_delta = this.x - x;
        int y_delta = this.y - y;
        int r = (int) Math.hypot(x_delta, y_delta);
        x_delta = (int) (r * Math.cos(imu.getAngularOrientation().firstAngle));
        y_delta = (int) (r * Math.sin(imu.getAngularOrientation().firstAngle));
        reset_encoders();
        x_pid.set_SP(x_delta);
        x_pid.set_SP(y_delta);

        int count = 0;
        int max_count = 20;

        while(((x_pid.get_PV() > x_pid.get_SP() + 1000 || x_pid.get_PV() < x_pid.get_SP() - 1000) ||
                (y_pid.get_PV() > y_pid.get_SP() + 1000 || y_pid.get_PV() < y_pid.get_SP() - 1000) ||
                (heading_pid.get_PV() > heading_pid.get_SP() + 0.05 || heading_pid.get_PV() <
                        heading_pid.get_SP() - 0.05)) && opModeIsActive()){

            x_pid.set_PV(wl.getCurrentPosition());
            y_pid.set_PV(wr.getCurrentPosition());
            heading_pid.set_PV(imu.getAngularOrientation().firstAngle);
            double rotation = heading_pid.get_PID();

            double ratio = (double) count / max_count;
            if(ratio < max_count){
                count ++;
                sleep(10);
                idle();
            }

            double x_pow = x_pid.get_PID();
            double y_pow = y_pid.get_PID();

            if((x_pid.get_PV() > x_pid.get_SP() + 400 || x_pid.get_PV() < x_pid.get_SP() - 400) && x_pow < FLOOR)
                x_pow = FLOOR;

            if((y_pid.get_PV() > y_pid.get_SP() + 400 || y_pid.get_PV() < y_pid.get_SP() - 400) && y_pow < FLOOR)
                y_pow = FLOOR;

            fl.setPower(ratio * Range.clip((-y_pow + x_pow + rotation), -1, 1));
            fr.setPower(ratio * Range.clip((y_pow + x_pow + rotation), -1, 1));
            br.setPower(ratio * Range.clip((y_pow - x_pow + rotation), -1, 1));
            bl.setPower(ratio * Range.clip((-y_pow - x_pow + rotation), -1, 1));
        }
        stop_motors();
        this.x += wl.getCurrentPosition();
        this.y += wr.getCurrentPosition();
    }


    /**
     * Utility method to set encoder values to zero to avoid slippage at larger values
     */
    private void reset_encoders(){
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Utility method to stop the robot
     */
    private void stop_motors(){
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    /**
     * Set the point of the robot; should be used to set the beginning of auto's coords.
     * This will be deprecate in version 2.0, where the robot aligns itself with a QR code.
     * @param x
     * @param y
     */
    protected void set_point(int x, int y){
        this.x = x;
        this.y = y;
    }

    protected void set_X(int x){
        this.x = x;
    }

    protected void set_Y(int y){
        this.y = y;
    }

}
