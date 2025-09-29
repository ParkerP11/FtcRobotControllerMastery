package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.lynx.LynxModule.*;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Path;


import java.util.ArrayList;


/*

Parker Pruitt, 2025
FTC team Ubett 8672

A autonomous library for movement based on position
using the Goblida Pinpoint Co-Processor

The coordinate System is (0,0) is the left corner of your starting side
Each tile is 24 inches
X is vertical forward backward axis
Y is the horizontal side to side axis
Standard (x,y) -> Our coordinates are (y,x)

 */
public class AutonomousDrive {


    //Error Tolerances
    public final double POS_ERROR_TOLERANCE = 0.01;
    public double POS_ERROR_TOLERANCE2 = 0.5;
    public double HEADING_ERROR_TOLERANCE = 0.01;

    private final double MAX_MOTOR_CURRENT = 9.2;
    private final double DEAD_WHEEL_RADIUS_MM = 16;

    public final double STRAFE_RATIO = 1;
    //PID controls

    //For Drive Movement
    public  double kDP = 1;
    public  double kDI = 0;
    public  double kDD = 0;
    public  double errorSumDX = 0;
    public double errorSumDY = 0;
    public double errorSumRangeD = 5;


    //For Turn Movement
    public  double kTP = 0.25;
    public  double kTI = 0.01;
    public  double kTD = 0.02;
    public  double errorSumT = 0;
    public  double errorSumRangeT = 25;



    //Drive motor names
    public String leftFrontName = "lf";
    public String leftBackName = "lb";
    public String rightFrontName = "rf";
    public String rightBackName = "rb";

    //Drive motor vars
    private static DcMotorEx lf;
    private static DcMotorEx lb;
    private static DcMotorEx rf;
    private static DcMotorEx rb;

    private static int leftFrontNum;
    private static int leftBackNum;
    private static int rightFrontNum;
    private static int rightBackNum;

    private static DcMotorControllerEx motorControllerEx;

    private static double[] motorCurrents = new double[4];

    //Gobilda Pinpoint
    private String odoName = "odo";
    private GoBildaPinpointDriver odo;

    //Pinpoint offsets from Center in mm and encoder Direction
    //Left is +X, Front is +Y
    private double xoffset = 150;
    private double yoffset = -19;

    private GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    private GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    //Dead Wheel Type

    private GoBildaPinpointDriver.GoBildaOdometryPods encoderPod = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    //LinearOpmode

    private LinearOpMode opMode;


    //Start Positions as {y,x} format and add to the master list for poses

    public double[] pos1 = new double[] {0,0, 0};
    public double[] pos2 = new double[] {8,36, 0 };


    private double[][] masterStartPoses = new double[][] { pos1, pos2};


    //Background Varables

    public static double timeLimit = 0;
    public static boolean outInfo = true;


    //Control hub and Expasion hub lnynx modules
    LynxModule controlHub, expansionHub;


    ArrayList<Path> paths = new ArrayList<>();






    //Default Constructor
    public AutonomousDrive(LinearOpMode opMode){
        this.opMode = opMode;

        controlHub = (LynxModule) opMode.hardwareMap.get(LynxModule.class, "Control Hub");
        expansionHub = (LynxModule) opMode.hardwareMap.get(LynxModule.class, "Expansion Hub");


        lf = opMode.hardwareMap.get(DcMotorEx.class, leftFrontName);
        lb = opMode.hardwareMap.get(DcMotorEx.class, leftBackName);
        rf = opMode.hardwareMap.get(DcMotorEx.class, rightFrontName);
        rb = opMode.hardwareMap.get(DcMotorEx.class, rightBackName);

        leftFrontNum = lf.getPortNumber();
        leftBackNum = lb.getPortNumber();
        rightFrontNum = rf.getPortNumber();
        rightBackNum = rb.getPortNumber();

        motorControllerEx = (DcMotorControllerEx)(lf.getController());

        motorCurrents = getMotorCurrents();

        motorControllerEx.setMotorCurrentAlert(leftFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(leftBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);

        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, odoName);

        //Pinpoint offsets from Center in mm
        //Left is +X, Front is +Y
        odo.setOffsets(xoffset, yoffset);
        odo.setEncoderDirections(xDirection, yDirection);
        odo.setEncoderResolution(encoderPod);

        odo.recalibrateIMU();
        opMode.sleep(250);
        odo.resetPosAndIMU();
        opMode.sleep(250);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, masterStartPoses[0][1],masterStartPoses[0][0], AngleUnit.DEGREES, masterStartPoses[0][0]));
        opMode.sleep(100);
    }

    //Constructor with different pose
    public AutonomousDrive(LinearOpMode opMode, int startPos){
        this.opMode = opMode;

        controlHub = (LynxModule) opMode.hardwareMap.get(LynxModule.class, "Control Hub");
        expansionHub = (LynxModule) opMode.hardwareMap.get(LynxModule.class, "Expansion Hub");

        lf = Robot.lf;
        lb = Robot.lb;
        rf = Robot.rf;
        rb = Robot.rb;
        /*
        leftFrontNum = lf.getPortNumber();
        leftBackNum = lb.getPortNumber();
        rightFrontNum = rf.getPortNumber();
        rightBackNum = rb.getPortNumber();

        motorControllerEx = (DcMotorControllerEx)(lf.getController());

        motorCurrents = getMotorCurrents();

        motorControllerEx.setMotorCurrentAlert(leftFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(leftBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);

         */


        odo = opMode.hardwareMap.get(GoBildaPinpointDriver.class, odoName);

        //Pinpoint offsets from Center in mm
        //Left is +X, Front is +Y
        odo.setOffsets(xoffset, yoffset);
        odo.setEncoderDirections(xDirection, yDirection);
        odo.setEncoderResolution(encoderPod);

        odo.recalibrateIMU();
        opMode.sleep(250);
        odo.resetPosAndIMU();
        opMode.sleep(250);

        odo.setPosition(new Pose2D(DistanceUnit.INCH, masterStartPoses[startPos][1],masterStartPoses[startPos][0], AngleUnit.DEGREES, masterStartPoses[startPos][0]));
        opMode.sleep(100);

    }

    public void updateTelemetry(){
        outputInfo();
    }

    //Getters and Setters and basic functions
    public static void drive(double rfPower, double rbPower, double lbPower, double lfPower) {

        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lb.setPower(lbPower);
        lf.setPower(lfPower);
    }

    public GoBildaPinpointDriver getPinPoint(){return odo; }

    public DcMotorEx getMotor(int num){
        switch (num){
            case 0:
                return lf;
            case 1:
                return lb;
            case 2:
                return rf;
            case 3:
                return rb;

        }
        return lf;
    }

    public Pose2D getPos(){return  odo.getPosition(); }
    public double getX(){return  -odo.getPosition().getX(DistanceUnit.INCH); }
    public double getY(){return  odo.getPosition().getY(DistanceUnit.INCH); }

    //getHeading outputs 0-360
    public double getHeading(){
        double rawHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        return  rawHeading + 180;
    }
    //getHeadingNorm outputs -180-180
    public double getHeadingNorm(){
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }
    //getHeadingUnNorm outputs -inf-inf
    public double getHeadingUnNorm(){
        return odo.getHeading();
    }


    public double getAngleToGo(double targetHeading){
        targetHeading = Math.abs(targetHeading) % 360;

        double currentHeading = getHeading();
        double angleTogo = targetHeading - currentHeading;
        currentHeading =  getHeading();

        angleTogo = targetHeading - currentHeading;

        if(Math.abs(angleTogo) > 180) {
            if (currentHeading < 180) {
                angleTogo = -((currentHeading) + (360 - targetHeading));
            } else {
                angleTogo = (targetHeading + (360 - currentHeading));
            }
        }
        return angleTogo;
    }

    public static double[] getMotorCurrents() {
        double currentLF = motorControllerEx.getMotorCurrent(leftFrontNum, CurrentUnit.AMPS);
        double currentLB = motorControllerEx.getMotorCurrent(leftBackNum, CurrentUnit.AMPS);
        double currentRF = motorControllerEx.getMotorCurrent(rightFrontNum, CurrentUnit.AMPS);
        double currentRB = motorControllerEx.getMotorCurrent(rightBackNum, CurrentUnit.AMPS);
        return new double[]{currentRF, currentRB, currentLB, currentLF};
    }

    public static String convertMotorCurrent(double[] list){
        String string = "";
        for(int i = 0; i < list.length; i++){
            string += ", " + list[i];
        }
        return string;
    }

    public static ArrayList<Boolean> isMotorsOver(){
        boolean overCurrentLF = motorControllerEx.isMotorOverCurrent(leftFrontNum);
        boolean overCurrentLB = motorControllerEx.isMotorOverCurrent(leftBackNum);
        boolean overCurrentRF = motorControllerEx.isMotorOverCurrent(rightFrontNum);
        boolean overCurrentRB = motorControllerEx.isMotorOverCurrent(rightBackNum);

        ArrayList<Boolean> list = new ArrayList<>();

        list.add(overCurrentLF);
        list.add(overCurrentLB);
        list.add(overCurrentRF);
        list.add(overCurrentRB);

        return list;
    }

    public double getVolts(){
        return controlHub.getInputVoltage(VoltageUnit.VOLTS);
    }

    public double limitPower(double power){
        if(getVolts() > 12){
            return (12/getVolts()) * power;
        }else{
            return power;
        }
    }

    public void setPID(double kP, double kI, double kD, int PIDNum){
        switch (PIDNum){
            case 0:
                kDP = kP;
                kDI = kI;
                kDD = kD;
                break;
            case 1:
                kTP = kP;
                kTI = kI;
                kTD = kD;
                break;

        }
    }


    //Set Limit to 0 if you don't want a time limit
    //Default is 0
    public static void setTimeLimit(double time){
        timeLimit = time;
    }

    public boolean checkTime(double startTime, double currentTime){
        if((currentTime - startTime) >= timeLimit){
            return true;
        }else {
            return false;
        }
    }

    public void setOutputInfo(boolean val){
        outInfo = val;
    }



    public void outputInfo(){
        if(outInfo){
            opMode.telemetry.addData("X pos: ", getX());
            opMode.telemetry.addData("Y pos: ", getY());
            opMode.telemetry.addData("Heading: ", getHeading());
            opMode.telemetry.addData("Heading Norm: ", getHeadingNorm());
            opMode.telemetry.addData("Heading UnNorm: ", odo.getHeading());
            opMode.telemetry.addData("Time: ", opMode.time);
            opMode.telemetry.addData("motor currents: ", convertMotorCurrent(getMotorCurrents()));
            opMode.telemetry.addData("Control Hub Power: ", getVolts());
            opMode.telemetry.update();
        }
    }



    public double turnSlope(double targetHeading, double startHeading, double currentDist, double startDist){
        return ((startHeading-targetHeading)/startDist) * (currentDist - startDist) + startHeading;
    }

    public double inchesToTicks(double dist){
        return (dist * 25.4 * 2000)/ (DEAD_WHEEL_RADIUS_MM * 2 * Math.PI);
    }

    public double ticksToInches(double dist){
        return (dist * (DEAD_WHEEL_RADIUS_MM * 2 * Math.PI))/(25.4 * 2000);
    }


    //PIDs

    public double movePID(double error, String axis){
        double output = error * kDP - error * kDD;
        if(Math.abs(error) <= errorSumRangeD){
            if(axis.toLowerCase().charAt(0) == 'y'){
                errorSumDY+= error;
                output += errorSumDY*kDI;
            }
            if(axis.toLowerCase().charAt(0) == 'x'){
                errorSumDX+= error;
                output += errorSumDX*kDI;
            }
        }
        return limitPower(output);
    }

    public double turnPID(double error){
        double output = error * kTP - error * kTD + errorSumT*kTI;
        if(Math.abs(error) <= errorSumRangeT){
            errorSumT += error;
        }
        return limitPower(output);
    }


    //Movement

    public void forward(double distance){
        double power = movePID(distance, "x");
        double startPose = odo.getEncoderX();
        double targetPose = inchesToTicks(distance) + startPose;
        while(targetPose - startPose> 5) {
            power = movePID(targetPose - startPose, "x");
            drive(power, power, power, power);
        }
        drive(0,0,0,0);
    }
    public void goToPointConstantHeading(double targetX, double targetY){
        odo.update();

        errorSumDX = 0;
        errorSumDY = 0;
        errorSumT = 0;

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = getHeading();
        double turn = turnPID(startHeading);

        double startTime = opMode.time;


        while(!checkTime(startTime,opMode.time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE || Math.abs(getAngleToGo(startHeading)) > HEADING_ERROR_TOLERANCE)
                && opMode.opModeIsActive()) {

            odo.update();
            outputInfo();

            targetXDist = -(targetX - getX());
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());

            double angleToGo = getAngleToGo(startHeading);

            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = movePID(targetXDist,"y"); // Remember, Y stick value is reversed
            double x = movePID(targetYDist,"x");
            double rx = turnPID(angleToGo);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
            double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

            rotX = rotX * STRAFE_RATIO;  // Counteract imperfect strafing

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
        opMode.sleep(50);
        drive(0,0,0,0);
    }

    public void goToPointLinear(double targetX, double targetY, double targetHeading){
        odo.update();

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);

        double startDist = totalDist;

        double startHeading = getHeading();
        double headingdist = targetHeading - startHeading;
        double currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);


        double startTime = opMode.time;


        while(!checkTime(startTime,opMode.time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE || Math.abs(getAngleToGo(currentTargetHead)) > HEADING_ERROR_TOLERANCE)
                && opMode.opModeIsActive()) {

            odo.update();
            outputInfo();


            targetXDist = targetX - getX();
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());
            currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);
            double angleToGo = getAngleToGo(currentTargetHead);

            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = -movePID(targetXDist, "y"); // Remember, Y stick value is reversed
            double x = movePID(targetYDist, "x");
            double rx = turnPID(angleToGo);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
            double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

            rotX = rotX * STRAFE_RATIO;  // Counteract imperfect strafing

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
        opMode.sleep(50);
        drive(0,0,0,0);
    }

    public void goToPointLinearNoStop(double targetX, double targetY, double targetHeading){
        odo.update();

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);

        double startDist = totalDist;

        double startHeading = getHeading();
        double headingdist = targetHeading - startHeading;
        double currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);


        double startTime = opMode.time;


        while(!checkTime(startTime,opMode.time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE2
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE2 || Math.abs(getAngleToGo(currentTargetHead)) > HEADING_ERROR_TOLERANCE)
                && opMode.opModeIsActive()) {

            odo.update();
            outputInfo();


            targetXDist = targetX - getX();
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());
            currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);
            double angleToGo = getAngleToGo(currentTargetHead);

            double v1 = 0;// lf
            double v2 = 0; // rf
            double v3 = 0; // lb
            double v4 = 0; // rb


            double y = -movePID(targetXDist, "y"); // Remember, Y stick value is reversed
            double x = movePID(targetYDist, "x");
            double rx = turnPID(angleToGo);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
            double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

            rotX = rotX * STRAFE_RATIO;  // Counteract imperfect strafing

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

    public void goToHeading(double heading){
        heading = Math.abs(heading) % 360;
        odo.update();
        double currentHeading = getHeading();
        double angleToGo = getAngleToGo(heading);
        double power;
        while(angleToGo > HEADING_ERROR_TOLERANCE){
            outputInfo();
            power = turnPID(angleToGo);
            drive(power, power,-power,-power);
        }
        opMode.sleep(50);
        drive(0,0,0,0);
    }


    public void createPath(ArrayList<Pose2D> points, ArrayList<Double> weights){
        Path path = new Path(this, opMode, points, weights);
        paths.add(path);
    }

    public Path getPath(int index){
        if(index < 0 || index >= paths.size()){
            return null;
        }
        return paths.get(index);
    }

    public void runPath(int pathNum){
        Path path = getPath(pathNum);
        double t = 0;
        while(!path.isComplete()){
            Pose2D point = path.getPoint(t);
            goToPointLinear(point.getX(DistanceUnit.INCH),point.getY(DistanceUnit.INCH), point.getHeading(AngleUnit.DEGREES));
            t += 0.1;
        }

    }

    public void runPath(int pathNum, double pace){
        Path path = getPath(pathNum);
        double t = 0;
        pace = Math.abs(pace);
        while(!path.isComplete()){
            Pose2D point = path.getPoint(t);
            goToPointLinearNoStop(point.getX(DistanceUnit.INCH),point.getY(DistanceUnit.INCH), point.getHeading(AngleUnit.DEGREES));
            t += pace;
        }
        drive(0,0,0,0);
        opMode.sleep(50);

    }
}


