package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class aFloor_Test extends Basic_Auto {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        floorMove(30_000, 0);
    }
}
