package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Red_Block extends Basic_Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        moveToBlocks(4.0);
        moveAlongBlocks(-12_000, 4.0, false);
        sleep(1000);
        grab_block();
        sleep(1000);
        floorMove(7_000, 0.5 * Math.PI);
        telemetry.addData("temp", wl.getCurrentPosition());
        specialFloorMove(34_000 + -wl.getCurrentPosition(), Math.PI);
        release();
        sleep(700);
        // floorMove(5000, 0.5 * Math.PI);
        floorMove(12_500, Math.PI);
    }
}
