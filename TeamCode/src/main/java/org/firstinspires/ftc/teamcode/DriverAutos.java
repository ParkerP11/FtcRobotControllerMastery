package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Robot.*;

public class DriverAutos {
    LinearOpMode opMode;
    GoBildaPinpointDriver odo;
    public DriverAutos(LinearOpMode opMode){
        this.opMode = opMode;
        odo = ad2.getPinPoint();
    }

    public void goToPointLinear(double targetX, double targetY, double targetHeading){
        odo.update();
        ad2.outputInfo();

        double targetXDist = -(targetX - ad2.getX());
        double targetYDist = targetY - ad2.getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);

        double currentHeadingRad = Math.toRadians(ad2.getHeadingNorm());

        double angleToGo = ad2.getAngleToGo(targetHeading);

        double v1 = 0;// lf
        double v2 = 0; // rf
        double v3 = 0; // lb
        double v4 = 0; // rb


        double y = ad2.movePID(targetXDist,"y"); // Remember, Y stick value is reversed
        double x = ad2.movePID(targetYDist,"x");
        double rx = ad2.turnPID(angleToGo);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
        double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

        rotX = rotX * ad2.STRAFE_RATIO;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        v1 = (rotY + rotX + rx) / denominator;
        v3 = (rotY - rotX + rx) / denominator;
        v2 = (rotY - rotX - rx) / denominator;
        v4 = (rotY + rotX - rx) / denominator;



        drive(v2,v4,v3,v1);
    }
}
