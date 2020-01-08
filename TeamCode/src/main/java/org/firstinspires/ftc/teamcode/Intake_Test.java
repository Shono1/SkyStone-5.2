package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Intake_Test extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "l");
        right = hardwareMap.get(DcMotor.class, "r");

        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a){
                left.setPower(1);
                right.setPower(-1);
            }
            if(gamepad1.b){
                left.setPower(-1);
                right.setPower(1);
            }
            else{
                left.setPower(0);
                right.setPower(0);
            }
        }
    }
}
