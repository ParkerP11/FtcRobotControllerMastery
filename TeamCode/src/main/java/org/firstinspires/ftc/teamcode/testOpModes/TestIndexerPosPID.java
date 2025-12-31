package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestIndexerPosPID extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initAll(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetryd = dashboard.getTelemetry();

        double[] incerments = new double[]{10,1,0.1,0.01, 0.001,0.0001};
        int indexInc = 0;
        int pose = 0;
        int i = 0;
        double power = 0.5;
        waitForStart();
        while(opModeIsActive()){
            if(Math.abs(indexer.indexerMotor.getCurrentPosition() - pose) <= 3) {
                i++;
                pose = (i * 50) * (int) (Math.pow(-1, i));
                indexer.indexerMotor.setTargetPosition(pose);
                indexer.indexerMotor.setPower(power);
            }

            if(gamepad1.aWasPressed()){
                indexInc = (indexInc + 1) % incerments.length;
            }
            if(gamepad1.dpadUpWasPressed()){
                indexer.kP += incerments[indexInc];
            }else if(gamepad1.dpadDownWasPressed()){
                indexer.kP -= incerments[indexInc];
            }

            if(gamepad1.dpadRightWasPressed()){
                indexer.kD += incerments[indexInc];
            }else if(gamepad1.dpadLeftWasPressed()){
                indexer.kD -= incerments[indexInc];
            }

            if(gamepad1.rightBumperWasPressed()){
                indexer.kF += incerments[indexInc];
            }else if(gamepad1.leftBumperWasPressed()){
                indexer.kF -= incerments[indexInc];
            }

            if(gamepad1.yWasPressed()){
                indexer.kI += incerments[indexInc];
            }else if(gamepad1.bWasPressed()){
                indexer.kI -= incerments[indexInc];
            }


            telemetryd.addData("Target Pose: ", pose);
            telemetryd.addData("Actual Pose: ", indexer.indexerMotor.getCurrentPosition());
            telemetryd.addData("Increment Amount: ", incerments[indexInc]);
            telemetryd.addData("KP: ", indexer.kP);
            telemetryd.addData("KI: ", indexer.kI);
            telemetryd.addData("KD: ", indexer.kD);
            telemetryd.addData("KF: ", indexer.kF);
            telemetryd.update();

        }
    }

    public PIDFCoefficients setNewPIDF(double kP, double kI , double kD, double kF){
        return new PIDFCoefficients(kP, kI,kD,kF);
    }
}
