package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Arrays;

public class Robot {
    public static LinearOpMode opMode;

    public static AutonomousDrive ad;

    public static GoBildaPinpointDriver odo;

    public static Intake intake;

    public static Controller c;

    public static ArtTracker artTrack;

    public static DcMotorEx lf, lb, rb, rf;
    public static DcMotorEx[] motors = new DcMotorEx[4];

    public static int[] motif = new int[3];

    public static HuskyLens huskyLens;

    /*Colors are
    0 --> no color
    1 --> Purple
    2 --> Green
     */


    public static void initAll(LinearOpMode opMode1){
        opMode = opMode1;
        intitDrive(opMode1);

        ad = new AutonomousDrive(opMode1);

        odo = ad.getPinPoint();

        intake = new Intake(opMode1);

        c = new Controller(opMode1);

        artTrack = new ArtTracker(opMode1);
    }

    public static void intitDrive(LinearOpMode opMode1){
        lf = opMode1.hardwareMap.get(DcMotorEx.class, "lf");
        lb = opMode1.hardwareMap.get(DcMotorEx.class, "lb");
        rb = opMode1.hardwareMap.get(DcMotorEx.class, "rb");
        rf = opMode1.hardwareMap.get(DcMotorEx.class, "rf");

        Arrays.stream(huskyLens.blocks()).anyMatch()

    }

    public static void drive(double rfPower, double rbPower, double lbPower, double lfPower) {
        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lb.setPower(lbPower);
        lf.setPower(lfPower);
    }

   public static void setIntake(int color) {
       if (!intake.wheelsRunning) {
           if (c.pad2.right_bumper && ! c.pad2Prev.right_bumper) {
               intake.intakeBall(color);
           }
       } else if (intake.wheelsRunning) {
           if (c.pad2.right_bumper && !c.pad2Prev.right_bumper) {
               intake.runWheels(0);
           }
       }
       c.update();
   }

    public static void setIntakeAuto() {
        setIntake(artTrack.getTargetArt());
    }

    public static void driveFC(){
        odo.update();
        c.update();

        if((Math.abs(c.pad1.left_stick_y) > 0.05 || Math.abs(c.pad1.left_stick_x) > 0.05 || Math.abs(c.pad1.right_stick_x) > 0.05)
                && opMode.opModeIsActive()){
        double currentHeadingRad = Math.toRadians(ad.getHeadingNorm());

        double v1 = 0;// lf
        double v2 = 0; // rf
        double v3 = 0; // lb
        double v4 = 0; // rb


        double y = -c.pad1.left_stick_y; // Remember, Y stick value is reversed
        double x = c.pad1.left_stick_x;
        double rx = c.pad1.right_stick_x;
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
        double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

        rotX = rotX * ad.STRAFE_RATIO;  // Counteract imperfect strafing

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
