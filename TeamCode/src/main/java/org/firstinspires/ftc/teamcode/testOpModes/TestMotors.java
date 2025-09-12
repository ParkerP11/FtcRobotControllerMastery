package org.firstinspires.ftc.teamcode.testOpModes;

import static org.firstinspires.ftc.teamcode.AutonomousDrive.setTimeLimit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.AutonomousDrive;

@TeleOp(name = "Motor Tester")
public class TestMotors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{

        setTimeLimit(4);

        DcMotor.RunMode[] mode = new DcMotor.RunMode[] {DcMotor.RunMode.RUN_TO_POSITION, DcMotor.RunMode.RUN_WITHOUT_ENCODER,DcMotor.RunMode.RUN_USING_ENCODER};
        Gamepad c1 = gamepad1;

        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");
        DcMotorEx lb = hardwareMap.get(DcMotorEx.class, "lb");
        DcMotorEx rb = hardwareMap.get(DcMotorEx.class, "rb");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(mode[1]);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int modeNum = 0;
        int motorNum = 0;



        waitForStart();
        while(opModeIsActive()){
            DcMotorEx motorEx;
            switch (motorNum){
                case 0: motorEx  = lf; break;
                case 1: motorEx  = rf; break;
                case 2: motorEx  = lb; break;
                case 3: motorEx  = rb; break;
                default:motorEx  = lf;
            }

            if(c1.right_bumper && motorNum < 3){
                motorNum++;
            }else if(c1.left_bumper && motorNum > 0){
                motorNum--;
            }
            if(c1.dpad_up && modeNum < 2){
                modeNum++;
            }else if(c1.dpad_down && modeNum > 0){
                modeNum--;
            }
            motorNum = Math.max(0, Math.min(2, modeNum));

            motorEx.setMode(mode[modeNum]);

            if(c1.a) {
                motorEx.setTargetPosition(0);
            }else if(c1.b){
                motorEx.setTargetPosition(500);
            }else if(c1.x){
                motorEx.setTargetPosition(250);
            }else if(c1.y){
                motorEx.setTargetPosition(1000);
            }

            if(Math.abs(c1.left_stick_y) > 0.01){
                motorEx.setPower(c1.left_stick_y);
            }else{
                motorEx.setPower(0);
            }
        }




    }






}
