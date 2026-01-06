package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Robot.*;
public class TestTelop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting... ");
        telemetry.update();
        initAll(this);
        telemetry.addLine("Ready! ");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            driveFC();

        }

    }
}
