package org.firstinspires.ftc.teamcode;

/**
 * Just a little test to show how cool and good Field Coordinate Auto is
 */
public class FC_Foundation_Test extends Field_Coordinate_Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        set_point(0, 0);
        move_to_point(20000, 20000);
    }
}
