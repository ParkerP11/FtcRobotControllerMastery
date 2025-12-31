package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Robot.*;

public class Indexer {
    //2 is green
    //1 is purple
    //0 is no Ball

    LinearOpMode opMode;

    private final double TICKS_PER_REV = 537.7;
    private final int ERROOR_TOLERANCE = 5;

    private double indexPower = 0.5;


    public DcMotorEx indexerMotor;

    
    int offset = (int)((TICKS_PER_REV/360) * 120);
    
    double outtakeAngle = 45;
    double intakeAngle = 315;

    double[][] slots = new double[3][4];

    int[][] motifPatterns = new int[][]{{1,1,2}, {1,2,1}, {2,1,1}};

    int[] motif = new int[3];

    int targetPose = 0;

    boolean loaderIsMoving = false;
    boolean indexerisMoving = false;

    ColorSensor localizeSensor;
    int[] tapeColor = new int[] {100,100,100};

    public double kP = 1;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0.1;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, kI,kD,kF);

    public Indexer(LinearOpMode opMode){
        this.opMode = opMode;

        indexerMotor = opMode.hardwareMap.get(DcMotorEx.class, "indexerMotor");
        indexerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerMotor.setTargetPosition(0);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        localizeSensor = opMode.hardwareMap.get(ColorSensor.class, "localizeSensor");


        updateSlots();

        loaclizeIndexer();
    }

    public void loaclizeIndexer(){
        indexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int[] diffColors = new int[] {localizeSensor.red() - tapeColor[0], localizeSensor.green() - tapeColor[1], localizeSensor.blue() - tapeColor[2]};
        int toggle = 0;
        while(toggle < 4) {
            if (diffColors[0] >= 0 && diffColors[1] >= 0 && diffColors[2] >= 0 && toggle == 0) {
                indexerMotor.setPower(-0.25);
                toggle++;
            } else if(diffColors[0] < 0 && diffColors[1] < 0 && diffColors[2] < 0 && toggle == 1) {
                indexerMotor.setPower(0.1);
                toggle++;
            }else if (diffColors[0] >= 0 && diffColors[1] >= 0 && diffColors[2] >= 0 && toggle == 2) {
                indexerMotor.setPower(0);
                toggle++;
            } else {
                indexerMotor.setPower(0.25);
            }
        }
        indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexerMotor.setTargetPosition(0);
        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexerMotor.setPower(0);
    }

    public void setMotif(int motifNum){
        motif = motifPatterns[motifNum];
    }

    public int[] getMotif(){
        return motif;
    }

    public void stopIndexer(){
        indexerisMoving = false;
        indexerMotor.setPower(0);
        updateSlots();
    }

    public void moveIndexer(int index, boolean atIntake){
        if(atIntake){
            updateSlots();
            double angleDiff = slots[index][2];
            moveIndexerHelper(angleDiff);
        }else {
            updateSlots();
            double angleDiff = slots[index][3];
            moveIndexerHelper(angleDiff);
        }
    }

    private void moveIndexerHelper(double angle){
        if(outtake.loaderAtRest()) {
            int pose = indexerMotor.getCurrentPosition();
            indexerMotor.setTargetPosition(pose + (int) (angle / 360.0 * TICKS_PER_REV));
            indexerisMoving = true;
            indexerMotor.setPower(indexPower);
            updateSlots();
        }else{
            indexerisMoving = false;
            outtake.setLoaderNearestRest();
            indexerMotor.setPower(0);
        }
    }

    public void updateSlots(){
        int pose = indexerMotor.getCurrentPosition();

        slots[0][1] = getAngle(pose);
        slots[1][1] = getAngle(pose + offset);
        slots[2][1] = getAngle(pose + 2 * offset);

        slots[0][2] = getAngleToGo(intakeAngle, slots[0][1]);
        slots[1][2] = getAngleToGo(intakeAngle, slots[1][1]);
        slots[2][2] = getAngleToGo(intakeAngle, slots[2][1]);

        slots[0][3] = getAngleToGo(outtakeAngle, slots[0][1]);
        slots[1][3] = getAngleToGo(outtakeAngle, slots[1][1]);
        slots[2][3] = getAngleToGo(outtakeAngle, slots[2][1]);
    }

    public void updateSlotColor(int index, int ballColor){
        slots[index][0] = ballColor;
    }

    public double getAngle(int ticks){
        if(ticks >= 0){
            return (ticks % TICKS_PER_REV)/360.0;
        }else{
            return 360.0 - (Math.abs(ticks) % TICKS_PER_REV)/360.0;
        }
    }

    public double getAngleToGo(double targetHeading, double currentHeading){
        targetHeading = Math.abs(targetHeading) % 360;

        double angleTogo = targetHeading - currentHeading;

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

    public int getTargetPose(int slotNum, boolean out){
        if(!out) {
            return (int) (targetPose + TICKS_PER_REV * (slots[slotNum][2] / 360));
        }else{
            return (int) (targetPose + TICKS_PER_REV * (slots[slotNum][3] / 360));
        }
    }

    public int getShortestDisttoIntake(){
        double angle1 = Math.abs(slots[0][2]);
        double angle2 = Math.abs(slots[1][2]);
        double angle3 = Math.abs(slots[2][2]);

        int[] order = new int[3];
        if(angle1 <= angle2 && angle1 <= angle3){
            order[0] = 0;
            if(angle2 <= angle3){
                order[1] = 1;
                order[2] = 2;
            }else{
                order[1] = 2;
                order[2] = 1;
            }
        }else if(angle2 <= angle1 && angle2 <= angle3){
            order[0] = 1;
            if(angle1 <= angle3){
                order[1] = 0;
                order[2] = 2;
            }else{
                order[1] = 2;
                order[2] = 0;
            }
        }else{
            order[0] = 2;
            if(angle1 <= angle2){
                order[1] = 0;
                order[2] = 1;
            }else{
                order[1] = 1;
                order[2] = 0;
            }
        }

        if(slots[order[0]][0]  == 0){
            return order[0];
        }else if(slots[order[1]][0]  == 0){
            return order[1];
        }else if(slots[order[2]][0]  == 0){
            return order[2];
        }else{
            return -1;
        }
    }

    public int getShortestDisttoOuttake(int ballColor){
        double angle1 = Math.abs(slots[0][3]);
        double angle2 = Math.abs(slots[1][3]);
        double angle3 = Math.abs(slots[2][3]);

        int[] order = new int[3];
        if(angle1 <= angle2 && angle1 <= angle3){
            order[0] = 0;
            if(angle2 <= angle3){
                order[1] = 1;
                order[2] = 2;
            }else{
                order[1] = 2;
                order[2] = 1;
            }
        }else if(angle2 <= angle1 && angle2 <= angle3){
            order[0] = 1;
            if(angle1 <= angle3){
                order[1] = 0;
                order[2] = 2;
            }else{
                order[1] = 2;
                order[2] = 0;
            }
        }else{
            order[0] = 2;
            if(angle1 <= angle2){
                order[1] = 0;
                order[2] = 1;
            }else{
                order[1] = 1;
                order[2] = 0;
            }
        }

        if(ballColor == 0){
            if(slots[order[0]][0]  > 0){
                return order[0];
            }else if(slots[order[1]][0]  > 0){
                return order[1];
            }else if(slots[order[2]][0]  > 0){
                return order[2];
            }else{
                return -1;
            }
        }
        if(slots[order[0]][0]  == ballColor){
            return order[0];
        }else if(slots[order[1]][0]  == ballColor){
            return order[1];
        }else if(slots[order[2]][0] == ballColor){
            return order[2];
        }else{
            return -1;
        }
    }

    public boolean getIndexerAtIntake(int slot){
        boolean out = (Math.abs(slots[slot][1] - intakeAngle) < ERROOR_TOLERANCE);
        return out;
    }

    public boolean getIndexerAtOuttake(int slot){
        boolean out = (Math.abs(slots[slot][1] - intakeAngle) < ERROOR_TOLERANCE);
        return out;
    }

    public int checkHasColor(int color){
        if(color == 1){
            int sum = 0;
            if(slots[0][0] == 1) sum += 1;
            if(slots[1][0] == 1) sum += 1;
            if(slots[2][0] == 1) sum += 1;
            return sum;
        }else{
            int sum = 0;
            if(slots[0][0] == 2) sum += 1;
            if(slots[1][0] == 2) sum += 1;
            if(slots[2][0] == 2) sum += 1;
            return sum;
        }
    }

    public boolean hasMotif(){
        return (checkHasColor(1) == 2 && checkHasColor(2) == 1);
    }

    public int[] getClosesttoOuttake(){
        updateSlots();
        if(Math.abs(slots[0][3]) >= Math.abs(slots[1][3]) && Math.abs(slots[0][3]) >= Math.abs(slots[2][3])){
            if(Math.abs(slots[1][3]) >= Math.abs(slots[2][3])){
                return new int[]{0,1,2};
            }
            return new int[]{0,2,1};
        }else if(Math.abs(slots[1][3]) >= Math.abs(slots[0][3]) && Math.abs(slots[1][3]) >= Math.abs(slots[2][3])){
            if(Math.abs(slots[0][3]) >= Math.abs(slots[2][3])){
                return new int[]{1,0,2};
            }
            return new int[]{1,2,0};
        }else{
            if(Math.abs(slots[0][3]) >= Math.abs(slots[1][3])){
                return new int[]{2,0,1};
            }
            return new int[]{2,1,0};
        }
    }

    public void indexMotif(int index){
        int slotNum = -1;
        int colorNum = motif[index];
        int[] slotNums = getClosesttoOuttake();
        if(slots[slotNums[0]][0] == colorNum){
            slotNum = slotNums[0];
        }else if(slots[slotNums[1]][0] == colorNum){
            slotNum = slotNums[1];
        }else if(slots[slotNums[2]][0] == colorNum){
            slotNum = slotNums[2];
        }
        if(slotNum >= 0) {
            moveIndexer(slotNum, false);
        }
    }
    public boolean isEmpty(){
        return (slots[0][0] == 0 && slots[1][0] == 0 && slots[2][0] == 0);
    }


}
