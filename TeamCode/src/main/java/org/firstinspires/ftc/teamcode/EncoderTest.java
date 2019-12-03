package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "test")
public class EncoderTest extends LinearOpMode {
    DcMotor fl;
    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "fl");
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("position", fl.getCurrentPosition());
            telemetry.update();
        }

    }
}
