package org.firstinspires.ftc.teamcode.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Robot.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Test movements")
public class TestMovement extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        initAll(this);
        ad.outputInfo();
        ad.setTimeLimit(2);

        ad.createPath(new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 180), 0);
        ad.getPath(0).addSpline(24,-24,180,90);
        ad.getPath(0).addSpline(48,0,180,180);
        ad.getPath(0).addSpline(-24,0,180,0);
        ad.getPath(0).addSpline(-24,-24,180,90);
        ad.getPath(0).addSpline(0,0,180,90);

        waitForStart();
        for(int i = 0; i < 50; i++){
            double x1 = ad.getPath(0).getPoint(i/50).getX(DistanceUnit.INCH);
            double x2 = ad.getPath(0).getPoint((i+1)/50).getX(DistanceUnit.INCH);
            double y1 = ad.getPath(0).getPoint(i/50).getY(DistanceUnit.INCH);
            double y2 = ad.getPath(0).getPoint((i+1)/50).getY(DistanceUnit.INCH);
            telemetry.addData("Dist " + i +  ": ", Math.hypot(x1-x2, y1-y2));
        }
        telemetry.update();
        sleep(5000);


        ad.runPath(0);

        telemetry.addLine("All Done!!");
        telemetry.update();
    }

}
