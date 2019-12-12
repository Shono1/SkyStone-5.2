package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="foundation_auto_red")
public class Foundation_Auto_Red extends Basic_Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        floorMove(7_000, Math.PI);
        floorMove(28_500, -0.5 * Math.PI);
        grab();

        floorMove(27_000, 0.5 * Math.PI);
        release();
        floorMove(3000, -0.5 * Math.PI);

        floorMove(40_000, Math.PI);
    }

}
