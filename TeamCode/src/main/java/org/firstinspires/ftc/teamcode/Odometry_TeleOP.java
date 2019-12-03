package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Field Centric With Odometry", group = "tests")
public class Odometry_TeleOP extends LinearOpMode {
    //private Gyroscope imu;
    private BNO055IMU imu;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    private DcMotor wr;
    private DcMotor wl;

    private Servo srv1;
    private Servo srv2;

    @Override
    public void runOpMode(){
        //Dc Motor Initialization
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        //Winch Initialization
        wr = hardwareMap.get(DcMotor.class, "wr");
        wl = hardwareMap.get(DcMotor.class, "wl");

        srv1 = hardwareMap.get(Servo.class, "grab1");
        srv2 = hardwareMap.get(Servo.class, "grab2");

        //IMU Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Odometry Variable Initialization
        VectorF controller_vector = new VectorF(0,0);
        VectorF robot_vector = new VectorF(0,0);
        float[] temp_orientation = {1, 0, 0, 1};
        GeneralMatrixF orientation = new GeneralMatrixF(2,2, temp_orientation);
        float robot_angle = 0;
        float[] temp = new float[4];
        PID_Controller pid_controller = new PID_Controller(1.5, 0.3, 0, 0); //Untuned: P = 3;
        double sp_angle = 0;

        waitForStart();
        while (opModeIsActive()){
            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            robot_angle = o.firstAngle;
            if (Double.isNaN(sp_angle)) {
                sp_angle = 0;
            }
            //sp_angle += 0.1 * gamepad1.right_stick_x;
//            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0){
//                sp_angle += 0;
//            }
//            else if(gamepad1.right_stick_x == 0) {
//                sp_angle += 0.03 * (Math.PI / 2);
//            }
//            else{
//                sp_angle += 0.09 * Math.atan(gamepad1.right_stick_y / gamepad1.right_stick_x);
//            }
            sp_angle += 0.05 * gamepad1.right_stick_x;

            pid_controller.set_SP(sp_angle);
            temp = rotation(robot_angle);
            pid_controller.set_PV(robot_angle);

            int a = -1;
            for(int i = 0; i < 2; i++){
                for(int k = 0; k < 2; k++) {
                    a++;
                    orientation.put(i, k, temp[a]);
                }
            }
            controller_vector.put(0, gamepad1.left_stick_x);
            controller_vector.put(1, gamepad1.left_stick_y);

            robot_vector = orientation.multiplied(controller_vector); //Magic happens here
            double pid_result = pid_controller.get_PID();

            fl.setPower(Range.clip((-robot_vector.get(1) * 0.01 + robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));
            fr.setPower(Range.clip((robot_vector.get(1) * 0.01 + robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));
            br.setPower(Range.clip((robot_vector.get(1) * 0.01 - robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));
            bl.setPower(Range.clip((-robot_vector.get(1) * 0.01 - robot_vector.get(0) * 0.01 - gamepad1.right_stick_x + 0.8 * pid_result), -1, 1));

            if(gamepad1.a){
                wr.setPower(1);
                wl.setPower(-1);
            }
            else if(gamepad1.b){
                wr.setPower(-1);
                wl.setPower(1);
            }
            else{
                wr.setPower(0);
                wl.setPower(0);
            }

            if(gamepad1.y){
                srv1.setPosition(0);
                srv2.setPosition(1);
            }
            if(gamepad1.x){
                srv1.setPosition(1);
                srv2.setPosition(0);
            }
            if(gamepad1.right_bumper){
                srv1.setPosition(0.5);
                srv2.setPosition(0.5);
            }
            telemetry.addData("angle", robot_angle);
            telemetry.addData("robot vec 0", robot_vector.get(0) * 0.01);
            telemetry.addData("robot vec 1", robot_vector.get(1) * 0.01);
            telemetry.addData("PID Result", pid_result);
            telemetry.addData("ArcTan", Math.atan(gamepad1.right_stick_y / gamepad1.right_stick_x));
            telemetry.addData("ArcTan Argument", gamepad1.right_stick_y / gamepad1.right_stick_x);
            telemetry.addData("Setpoint", pid_controller.get_SP());
            telemetry.update();
        }
    }

    public static float[] rotation(float angle){
        float[] ret = {to_degrees((float) Math.cos(angle)), -to_degrees((float) Math.sin(angle)),
                to_degrees((float) Math.sin(angle)), to_degrees((float) Math.cos(angle))};
        return ret;
    }

    public static float to_degrees(float angle){
        return (float) (angle * 180 / Math.PI);
    }

}
