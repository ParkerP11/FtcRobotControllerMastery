package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive;
import static org.firstinspires.ftc.teamcode.AutonomousDrive.*;


@Autonomous(name = "PID TUNER")
public class TestPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        AutonomousDrive ad = new AutonomousDrive(this, 1);
        setTimeLimit(4);
        ad.setOutputInfo(true);
        kI =


    }
}
