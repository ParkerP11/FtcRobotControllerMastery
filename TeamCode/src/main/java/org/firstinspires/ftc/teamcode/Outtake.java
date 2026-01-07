package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Robot.*;

public class Outtake{
    LinearOpMode opMode;
    Servo loader, headingServo;
    DcMotorEx launchMotor1,launchMotor2;

    public final double ONE_SERVO_REV = 0.2;
    public final int HEADING_SERVO_RANGE = 1800;

    double launchSpeed = 1600;

    double restPose = 0;

    double loadPose = 0;
    double launchPose = 1;

    boolean loadedBall = false;
    boolean loaderInIndex  = false;

    int launchSpeedDiffRange = 500;

    double launchSpeedPrev = 0;

    int side = 0;

    double targetHeadingAngle = 0.5;

    int motifIndex = 0;

    double range1 = 51;
    double range2 = 67;
    double range3 = 75;
    double launchSpeed1 = 1112;
    double launchSpeed2 = 1210;
    double launchSpeed3 = 2010;
    double loaderStartTime = 0;

    public Outtake(LinearOpMode opMode, boolean isTelop, int side){
        this.opMode = opMode;
        loader = opMode.hardwareMap.get(Servo.class, "Loader");
        headingServo = opMode.hardwareMap.get(Servo.class, "headingServo");
        launchMotor1 = opMode.hardwareMap.get(DcMotorEx.class, "launchMotor1");
        launchMotor2 = opMode.hardwareMap.get(DcMotorEx.class, "launchMotor2");
        this.side = side;

        if(!isTelop) {
            loader.setPosition(restPose);
            headingServo.setPosition(0.5);
            opMode.sleep(500);
        }
    }

    public boolean loaderAtRest(){
         return (loader.getPosition()  % ONE_SERVO_REV == 0);
    }

    public boolean motorsAtSpeed(double speed){
        return (launchMotor1.getVelocity() >= speed && launchMotor1.getVelocity() >= speed);
    }

    public double getMeanLaunchSpeed(){
        return (launchMotor1.getVelocity() + launchMotor2.getVelocity())/2.0;
    }

    public boolean ballLaunched(){
        return (launchSpeedPrev - getMeanLaunchSpeed() >= launchSpeedDiffRange);
    }
    public void alignHeadingServo(){
        targetHeadingAngle = getHeadingAngle();
        headingServo.setPosition(targetHeadingAngle/HEADING_SERVO_RANGE);
    }

    public double getHeadingAngle(){
        if(side == 0){
            double diffx, diffy;
            diffx = targetPosBlue[0] - ad2.getY();
            diffy = targetPosBlue[1] - ad2.getX();
            double angle = Math.atan2(diffy, diffx);
            angle = Math.toDegrees(Math.abs(angle)) + 180;
            return angle % 360;
        }else if(side == 1){
            double diffx, diffy;
            diffx = targetPosBlue[0] - ad2.getY();
            diffy = targetPosBlue[1] - ad2.getX();
            double angle = Math.atan2(diffy, diffx);
            angle = Math.toDegrees(Math.abs(angle)) + 180;
            return angle % 360;
        }else{
            return ad2.getHeading();
        }
    }

    public void setLoaderNearestRest(){
        double pose = loader.getPosition();
        if(pose % 0.2 <= 0.05){
            double newPose = pose - pose % 0.2;
            newPose = Math.max(0, Math.min(1,newPose));
            loader.setPosition(newPose);
        }else{
            double newPose = pose +(0.2-pose % 0.2);
            newPose = Math.max(0, Math.min(1,newPose));
            loader.setPosition(newPose);
        }
    }

    public void loadBall(){
        loader.setPosition(launchPose);
        indexer.stopIndexer();
        loadedBall = true;
        loaderStartTime = opMode.getRuntime();
    }

    public void resetServo(){
        loader.setPosition(loadPose);
        indexer.indexerMotor.setPower(0);
        indexer.indexerisMoving = false;
    }
    public void setMotorsSpeed(double speed){
        launchMotor1.setVelocity(speed);
        launchMotor2.setVelocity(speed);
    }

    public boolean motorsatLaunchSpeed(double speed){
        return (launchMotor1.getVelocity() >= speed && launchMotor2.getVelocity() >= speed);
    }

    public double getTargetLaunchSpeed(){
        if(side == 0){
            if(Math.hypot(targetPosBlue[0] - ad2.getX(), targetPosBlue[1] - ad2.getY()) <= range1){
                return launchSpeed1;
            }
            if(Math.hypot(targetPosBlue[0] - ad2.getX(), targetPosBlue[1] - ad2.getY()) <= range2){
                return launchSpeed2;
            }
            return launchSpeed3;
        }else if(side == 1){
            if(Math.hypot(targetPosRed[0] - ad2.getX(), targetPosRed[1] - ad2.getY()) <= range1){
                return launchSpeed1;
            }
            if(Math.hypot(targetPosRed[0] - ad2.getX(), targetPosRed[1] - ad2.getY()) <= range2){
                return launchSpeed2;
            }
            return launchSpeed3;
        }else{
            if(Math.hypot(targetPosRed[0] - ad2.getX(), targetPosRed[1] - ad2.getY()) <= range1){
                return launchSpeed1;
            }
            if(Math.hypot(targetPosRed[0] - ad2.getX(), targetPosRed[1] - ad2.getY()) <= range2){
                return launchSpeed2;
            }
            return launchSpeed3;
        }
    }

    public boolean checkPose(){
        return (Math.abs(ad2.getY()) - 5 >= ad2.getX() || -Math.abs(ad2.getY()) - 48 <= ad2.getX());
    }
    public void launchBallTelop() {
        /*
        if(opMode.gamepad2.a){
            setMotorsSpeed(launchSpeed);
            int[] allowedColors = intake.getNeededMotifColors();
            int targetColor = 0;
            if(allowedColors[0] > 0){
                targetColor = 2;
            }else if(allowedColors[1] > 0){
                targetColor = 1;
            }
            if(targetColor > 0) {
                int index = indexer.getShortestDisttoOuttake(targetColor);
                if (index >= 0) {
                    if (motorsAtSpeed(launchSpeed) && indexer.getIndexerAtOuttake(index)) {
                        indexer.stopIndexer();
                        if (!loadedBall) {
                            loadBall();
                            loadedBall = true;
                        }
                    } else {
                        indexer.moveIndexer(index, false);
                    }
                }
            }
            else{
                resetServo();
            }
        }else if(opMode.gamepad2.x){
            setMotorsSpeed(launchSpeed);

            int index = indexer.getShortestDisttoOuttake(0);
            if(index >= 0) {
                if (motorsAtSpeed(launchSpeed) && indexer.getIndexerAtOuttake(index)) {
                    indexer.stopIndexer();
                    if (!loadedBall) {
                        loadBall();
                        loadedBall = true;
                    }
                } else {
                    indexer.moveIndexer(index, false);
                }
            }
        }else{
            setMotorsSpeed(0);
            resetServo();
        }

        if(ballLaunched()){
            loadedBall = false;
        }
        launchSpeedPrev = getMeanLaunchSpeed();

         */

        upDateOuttake();
        if (opMode.gamepad2.a) {      //launches motif
            setMotorsSpeed(getTargetLaunchSpeed());
            if (indexer.hasMotif()) {
                if(!loadedBall){
                    indexer.indexMotif(motifIndex);
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else if (opMode.gamepad2.x) {        //launches all balls
            setMotorsSpeed(getTargetLaunchSpeed());
            if (!indexer.isEmpty()) {
                if(!loadedBall){
                    int[] indexes = indexer.getClosesttoOuttake();
                    if(indexer.slots[indexes[0]][0] > 0){
                        indexer.getIndexerAtOuttake(indexes[0]);
                    }else  if(indexer.slots[indexes[1]][0] > 0){
                        indexer.getIndexerAtOuttake(indexes[1]);
                    }else if(indexer.slots[indexes[2]][0] > 0){
                        indexer.getIndexerAtOuttake(indexes[2]);
                    }
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else if (opMode.gamepad2.x) {        // launches purple
            setMotorsSpeed(getTargetLaunchSpeed());
            if (!indexer.isEmpty()) {
                if(!loadedBall){
                    int index = indexer.getShortestDisttoOuttake(1);
                    indexer.getIndexerAtOuttake(index);
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else if (opMode.gamepad2.y) {        // launches green
            setMotorsSpeed(getTargetLaunchSpeed());
            if (!indexer.isEmpty()) {
                if(!loadedBall){
                    int index = indexer.getShortestDisttoOuttake(1);
                    indexer.getIndexerAtOuttake(index);
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else {
            setMotorsSpeed(0);
        }
    }

    public void launchBall(int color) {
        /*
        if(opMode.gamepad2.a){
            setMotorsSpeed(launchSpeed);
            int[] allowedColors = intake.getNeededMotifColors();
            int targetColor = 0;
            if(allowedColors[0] > 0){
                targetColor = 2;
            }else if(allowedColors[1] > 0){
                targetColor = 1;
            }
            if(targetColor > 0) {
                int index = indexer.getShortestDisttoOuttake(targetColor);
                if (index >= 0) {
                    if (motorsAtSpeed(launchSpeed) && indexer.getIndexerAtOuttake(index)) {
                        indexer.stopIndexer();
                        if (!loadedBall) {
                            loadBall();
                            loadedBall = true;
                        }
                    } else {
                        indexer.moveIndexer(index, false);
                    }
                }
            }
            else{
                resetServo();
            }
        }else if(opMode.gamepad2.x){
            setMotorsSpeed(launchSpeed);

            int index = indexer.getShortestDisttoOuttake(0);
            if(index >= 0) {
                if (motorsAtSpeed(launchSpeed) && indexer.getIndexerAtOuttake(index)) {
                    indexer.stopIndexer();
                    if (!loadedBall) {
                        loadBall();
                        loadedBall = true;
                    }
                } else {
                    indexer.moveIndexer(index, false);
                }
            }
        }else{
            setMotorsSpeed(0);
            resetServo();
        }

        if(ballLaunched()){
            loadedBall = false;
        }
        launchSpeedPrev = getMeanLaunchSpeed();

         */

        upDateOuttake();
        if (color == 0) {      //launches motif
            setMotorsSpeed(getTargetLaunchSpeed());
            if (indexer.hasMotif()) {
                if(!loadedBall){
                    indexer.indexMotif(motifIndex);
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else if (color == -1) {        //launches all balls
            setMotorsSpeed(getTargetLaunchSpeed());
            if (!indexer.isEmpty()) {
                if(!loadedBall){
                    int[] indexes = indexer.getClosesttoOuttake();
                    if(indexer.slots[indexes[0]][0] > 0){
                        indexer.getIndexerAtOuttake(indexes[0]);
                    }else  if(indexer.slots[indexes[1]][0] > 0){
                        indexer.getIndexerAtOuttake(indexes[1]);
                    }else if(indexer.slots[indexes[2]][0] > 0){
                        indexer.getIndexerAtOuttake(indexes[2]);
                    }
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else if (color == 1) {        // launches purple
            setMotorsSpeed(getTargetLaunchSpeed());
            if (!indexer.isEmpty()) {
                if(!loadedBall){
                    int index = indexer.getShortestDisttoOuttake(1);
                    indexer.getIndexerAtOuttake(index);
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else if (color == 2) {        // launches green
            setMotorsSpeed(getTargetLaunchSpeed());
            if (!indexer.isEmpty()) {
                if(!loadedBall){
                    int index = indexer.getShortestDisttoOuttake(1);
                    indexer.getIndexerAtOuttake(index);
                    if(!indexer.indexerisMoving && checkPose() && motorsatLaunchSpeed(getTargetLaunchSpeed())){
                        loadBall();
                    }
                }
            }
        } else {
            setMotorsSpeed(0);
        }
    }



    public void upDateOuttake(){
        if(loadedBall && (opMode.getRuntime() - loaderStartTime) > 0.5){
            loadedBall = false;
        }else if(loadedBall && (opMode.getRuntime() - loaderStartTime) > 0.25){
            loader.setPosition(loadPose);
        }
        if(loader.getPosition() >= 0.9 && !loadedBall &&!indexer.indexerisMoving){
            resetServo();
        }

        headingServo.setPosition(targetHeadingAngle/HEADING_SERVO_RANGE);


    }



}
