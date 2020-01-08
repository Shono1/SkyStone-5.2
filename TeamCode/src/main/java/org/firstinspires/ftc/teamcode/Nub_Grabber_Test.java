package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Nub_Grabber_Test extends LinearOpMode {
    private Servo grab;

    @Override
    public void runOpMode() throws InterruptedException {
        grab = hardwareMap.get(Servo.class, "srv0");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) grab.setPosition(0);
            if (gamepad1.b) grab.setPosition(1);
        }
    }
}
