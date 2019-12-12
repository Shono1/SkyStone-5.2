package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Block_Auto extends Basic_Auto {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
//        polarMove(10_000, -0.5 * Math.PI);
//        if(sense_block())
//
//
//        while(opModeIsActive()){
//            telemetry.addData("L_Red", clr_l.red());
//            telemetry.addData("L_green", clr_l.green());
//            telemetry.addData("L_blue", clr_l.blue());
//
//            telemetry.addData("R_Red", clr_r.red());
//            telemetry.addData("R_green", clr_r.green());
//            telemetry.addData("R_blue", clr_r.blue());
//
//            if(sense_block()) telemetry.addData("block", "sensed");
//            else telemetry.addData("block", "not sensed");
//            telemetry.update();
//        }
        polarMove(10_000, -0.5 * Math.PI);
        polarMove(7_000, Math.PI);
        move_and_sense();
        sleep(800);
        grab_block();
        sleep(700);
        polarMove(5000, 0.5 * Math.PI);
        polarMove(39_000, 0);
        release();
        sleep(1000);
        polarMove(7000, Math.PI);



    }
}
