package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Launcher Test")
public class LauncherTester extends LinearOpMode {
    final double  TICKS_PER_REV = 28;//28 ticks per rev for a 6k rpm motor

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //have a var that we can control that will be used for the speed
        double rpm = 0;
        double incrementSpeed = 10;
        boolean runMotor = false;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                rpm += incrementSpeed;
            }else if(gamepad1.left_bumper){
                rpm -= incrementSpeed;
            }

            if(runMotor){
                launchMotor.setVelocity(getTickSpeed(rpm));
            }else{
                launchMotor.setVelocity(0);
            }

            telemetry.addData("Motor Rpm: ", launchMotor.getVelocity()/TICKS_PER_REV*60);
            telemetry.addData("Target Rpm: ", rpm);
            telemetry.update();

        }
    }

    public double getTickSpeed(double speed){
        return speed*TICKS_PER_REV/60;
    }
}
