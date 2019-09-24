package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Field Centric w/ PID", group = "tests")
public class field_centric_with_rotation_PID extends LinearOpMode {
    //private Gyroscope imu;
    private BNO055IMU imu;
    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    @Override
    public void runOpMode(){
        //Dc Motor Initialization
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

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

        waitForStart();
        while (opModeIsActive()){
            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            robot_angle = o.firstAngle;
            temp = rotation(robot_angle);
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

            fl.setPower(Range.clip((-robot_vector.get(1) * 0.01 + robot_vector.get(0) * 0.01 - gamepad1.right_stick_x), -1, 1));
            fr.setPower(Range.clip((robot_vector.get(1) * 0.01 + robot_vector.get(0) * 0.01 - gamepad1.right_stick_x), -1, 1));
            br.setPower(Range.clip((robot_vector.get(1) * 0.01 - robot_vector.get(0) * 0.01 - gamepad1.right_stick_x), -1, 1));
            bl.setPower(Range.clip((-robot_vector.get(1) * 0.01 - robot_vector.get(0) * 0.01 - gamepad1.right_stick_x), -1, 1));

            telemetry.addData("angle", robot_angle);
            telemetry.addData("robot vec 0", robot_vector.get(0) * 0.01);
            telemetry.addData("robot vec 1", robot_vector.get(1) * 0.01);
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
