package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.c;
import static org.firstinspires.ftc.teamcode.Robot.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    LinearOpMode opMode;
    Gamepad pad1, pad2, pad1Prev, pad2Prev;

    public Controller(LinearOpMode opMode){
        pad1 = opMode.gamepad1;
        pad2 = opMode.gamepad2;

        pad1Prev = pad1;
        pad2Prev = pad2;
    }

    public void update(){
        pad1Prev = pad1;
        pad2Prev = pad2;
        pad1 = opMode.gamepad1;
        pad2 = opMode.gamepad2;
    }






}
