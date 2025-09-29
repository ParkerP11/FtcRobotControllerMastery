package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousDrive;

@Autonomous(name = "Strafe Ratio Finder")
public class StrafRatioFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDrive ad = new AutonomousDrive(this);

        int distance = 72;

        ad.goToPointConstantHeading(0,distance);


    }
}
