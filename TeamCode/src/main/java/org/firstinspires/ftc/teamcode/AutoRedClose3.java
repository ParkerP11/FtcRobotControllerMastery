package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.ad2;
import static org.firstinspires.ftc.teamcode.Robot.da;
import static org.firstinspires.ftc.teamcode.Robot.drive;
import static org.firstinspires.ftc.teamcode.Robot.indexer;
import static org.firstinspires.ftc.teamcode.Robot.initAll;
import static org.firstinspires.ftc.teamcode.Robot.intake;
import static org.firstinspires.ftc.teamcode.Robot.outtake;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Red Close 3")
public class AutoRedClose3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Starting...");
        telemetry.update();
        initAll(this);
        ad2.setOutputInfo(true);
        ad2.setTimeLimit(3.5);
        ad2.resetOdo(this, 48.919, -51.185, 123.855);

        outtake.upDateOuttake();
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        //Launch Motif 1
        ad2.goToPointConstantHeading(36,36);

        while(!outtake.ballLaunched()){
            outtake.launchBall(0);
        }

        //Get Motif 2
        indexer.getIndexerAtIntake(indexer.getShortestDisttoIntake());

        intake.intakeBall(false);
        ad2.goToPointLinear(12,-18,90);
        intakeBalls( 12, -50);

        //Launch Motif 2
        outtake.upDateOuttake();
        ad2.goToPointConstantHeading(36,36);
        outtake.upDateOuttake();

        while(!outtake.ballLaunched()){
            outtake.upDateOuttake();
            outtake.launchBall(0);
        }

        //Get Motif 3
        indexer.getIndexerAtIntake(indexer.getShortestDisttoIntake());

        intake.intakeBall(false);
        ad2.goToPointLinear(0,-18,90);
        intakeBalls( 0, -50);

        //Launch Motif 3
        outtake.upDateOuttake();
        ad2.goToPointConstantHeading(36,36);
        outtake.upDateOuttake();

        while(!outtake.ballLaunched()){
            outtake.upDateOuttake();
            outtake.launchBall(0);
        }

    }

    public void intakeBalls(double x, double y){
        while(Math.abs(ad2.getX() - x) > ad2.POS_ERROR_TOLERANCE && Math.abs(ad2.getY() - y) > ad2.POS_ERROR_TOLERANCE){
            intake.intakeBall(false);
            da.goToPointLinear(x,y, ad2.getHeading());
        }
    }
}
