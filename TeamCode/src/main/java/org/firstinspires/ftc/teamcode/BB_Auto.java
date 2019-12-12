package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue basic")
public class BB_Auto extends Basic_Auto {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        polarMove(30_000, Math.PI);
    }
}
