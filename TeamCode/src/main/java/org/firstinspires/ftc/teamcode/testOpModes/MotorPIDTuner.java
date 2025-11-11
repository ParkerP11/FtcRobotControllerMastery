package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "PID Tester")
public class MotorPIDTuner extends LinearOpMode {
    final double  TICKS_PER_REV = 28;//28 ticks per rev for a 6k rpm motor

    @Override
    public void runOpMode() throws InterruptedException {


        //Setup motor
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        Setup the motor controller for the motor.
        Have to cast the DcMotorControllerEx since launchMotor.getController()
        returns DcMotorController and not DcMotorControllerEx
         */
        DcMotorControllerEx dcMotorControllerEx = (DcMotorControllerEx)(launchMotor.getController());

        //Get the port number of the motor we want
        int motorPortNum = launchMotor.getPortNumber();


        //Our PIDF Coefficents
        double kp,ki,kd, kf= 0;

        //Get the motor actual PIDF Coefficents when it is in RUN_USING_ENCODER
        PIDFCoefficients pidNumOrg = dcMotorControllerEx.getPIDFCoefficients(motorPortNum, DcMotor.RunMode.RUN_USING_ENCODER);

        kp = pidNumOrg.p;;
        ki = pidNumOrg.i;;
        kd = pidNumOrg.d;;

        //I don't recommend changing kf, it's just make the input signal for the rest of PID more clear
        kf = pidNumOrg.f;

        //have a var that we can control that will be used for the speed
        double rpm = 0;
        double incrementSpeed = 10;
        double pidfIncrement = 0.001;
        //So we can turn on and off the motor without the program stopping
        boolean runMotor = false;
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                rpm += incrementSpeed;
            }else if(gamepad1.left_bumper){
                rpm -= incrementSpeed;
            }

            if(gamepad1.dpad_up){
                kp += pidfIncrement;
            }else if(gamepad1.dpad_down){
                kp -= pidfIncrement;
            }

            if(gamepad1.dpad_right){
                ki += pidfIncrement;
            }else if(gamepad1.dpad_left){
                ki -= pidfIncrement;
            }

            if(gamepad1.a){
                kd += pidfIncrement;
            }else if(gamepad1.x){
                kd -= pidfIncrement;
            }



            PIDFCoefficients updatedPIDF = pidNumOrg;
            updatedPIDF.p = kp;
            updatedPIDF.i = ki;
            updatedPIDF.d = kd;

           dcMotorControllerEx.setPIDFCoefficients(motorPortNum, DcMotor.RunMode.RUN_USING_ENCODER, updatedPIDF);


            if(runMotor){
                launchMotor.setVelocity(getTickSpeed(rpm));
            }else{
                launchMotor.setVelocity(0);
            }

            telemetry.addData("Motor Rpm: ", launchMotor.getVelocity()/TICKS_PER_REV*60);
            telemetry.addData("Target Rpm: ", rpm);
            telemetry.addData("kP: ", kp);
            telemetry.addData("kI: ", ki);
            telemetry.addData("kD: ", kd);
            telemetry.update();



        }
    }

    public double getTickSpeed(double speed){
        return speed*TICKS_PER_REV/60;
    }
}
