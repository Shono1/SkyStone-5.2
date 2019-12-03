package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

@Autonomous(name = "odometry test")
public class Odometry_Auto_Test extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    private DcMotor wr;
    private DcMotor wl;
    private DcMotor dummy;

    private Servo srv1;
    private Servo srv2;

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
        dummy = hardwareMap.get(DcMotor.class, "dummy");

//        srv1 = hardwareMap.get(Servo.class, "grab1");
//        srv2 = hardwareMap.get(Servo.class, "grab2");

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
        PID_Controller pid_controller = new PID_Controller(1.5, 0.3, 0, 0); //Untuned: P = 3;
        double sp_angle = 0;

        // Odometer initialization
        PID_Controller x_pid = new PID_Controller(1, 1, 0, 1); // TODO: tune this
        PID_Controller y_pid = new PID_Controller(1, 1, 0, 1); // TODO: tune this
        PID_Controller r_pid = new PID_Controller(1, 1, 0, 1); // TODO: tune this

        waitForStart();
        // Drive in a line
        x_pid.set_SP(0);
        x_pid.set_PV(0);

        y_pid.set_SP(10_000);
        y_pid.set_PV(0);

        r_pid.set_SP(0);
        r_pid.set_PV(0);

        // TODO: Decide whether to use r_pid or imu for heading control
        wr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dummy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(((x_pid.get_PV() > x_pid.get_SP() + 50 || x_pid.get_PV() < x_pid.get_SP() - 50) ||
                (y_pid.get_PV() > y_pid.get_SP() + 50 || y_pid.get_PV() < y_pid.get_SP() - 50) ||
                (r_pid.get_PV() > r_pid.get_SP() + 50 || r_pid.get_PV() < r_pid.get_SP() - 50)) && opModeIsActive()){
            double x, y, r;
            x = x_pid.get_PID();
            y = y_pid.get_PID();
            r = r_pid.get_PID();

//            fl.setPower(Range.clip((-y + x - r), -1, 1));
//            fr.setPower(Range.clip((y + x - r), -1, 1));
//            br.setPower(Range.clip((y - x - r), -1, 1));
//            bl.setPower(Range.clip((-y - x - r), -1, 1));


//            x_pos = wr.getCurrentPosition() - (wr.getCurrentPosition() + dummy.getCurrentPosition());
//            y_pos = wl.getCurrentPosition() - (wl.getCurrentPosition() + dummy.getCurrentPosition());
//            r_pos = dummy.getCurrentPosition() + wl.getCurrentPosition();
            int x_pos, y_pos, r_pos;

//            x_pos = (wr.getCurrentPosition() + (dummy.getCurrentPosition() + wr.getCurrentPosition()));
//            y_pos = (wl.getCurrentPosition() + (dummy.getCurrentPosition() + wl.getCurrentPosition()));
//            r_pos = dummy.getCurrentPosition() + wl.getCurrentPosition();

            x_pos = (wr.getCurrentPosition() - wl.getCurrentPosition());
            y_pos = (wl.getCurrentPosition() - wr.getCurrentPosition());
            r_pos = dummy.getCurrentPosition() + wl.getCurrentPosition();

            x_pid.set_PV(x_pos); // x port 0
            y_pid.set_PV(y_pos); // y port 1
            r_pid.set_PV(r_pos); // r port 2

            telemetry.addData("x calculated", x_pos);
            telemetry.addData("y calculated", y_pos);
            telemetry.addData("r calculated", r_pos);

            telemetry.addData("x ticks", wr.getCurrentPosition());
            telemetry.addData("y ticks", wl.getCurrentPosition());
            telemetry.addData("r ticks", dummy.getCurrentPosition());

            telemetry.update();

            idle();
        }
//        fl.setPower(Range.clip((-robot_vector.get(1) * 0.01 + robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));
//        fr.setPower(Range.clip((robot_vector.get(1) * 0.01 + robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));
//        br.setPower(Range.clip((robot_vector.get(1) * 0.01 - robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));
//        bl.setPower(Range.clip((-robot_vector.get(1) * 0.01 - robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));




    }

    public int[] getEncoderPositions(){
        int[] ret = {fl.getCurrentPosition(), bl.getCurrentPosition(), fr.getCurrentPosition()};
        return ret;
    }

}
