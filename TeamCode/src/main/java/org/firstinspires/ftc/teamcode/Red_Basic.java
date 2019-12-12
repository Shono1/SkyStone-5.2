package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="red basic")
public class Red_Basic extends Basic_Auto {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        polarMove(30_000, 0);
    }
}
