package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Robot.*;

public class Intake {

    Servo intakeServo1;
    ColorSensor colorSens1, colorSens2, colorSens3;
    LinearOpMode opMode;

    final int[] PURPLE = new int[]{128,0,128};
    final int[] GREEN = new int[]{0,256,0};

    public int[] normalizedReading1, normalizedReading2 = new int[3];

    final int COLOR_RANGE = 25;

    boolean wheelsRunning = false;

    int motifNum = 0;

    int ballColoratIntake = 0;


    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
        intakeServo1 = opMode.hardwareMap.get(Servo.class, "intakeServo1");
        colorSens1 = opMode.hardwareMap.get(ColorSensor.class, "colorSens1");
        colorSens2 = opMode.hardwareMap.get(ColorSensor.class, "colorSens2");
        colorSens3 = opMode.hardwareMap.get(ColorSensor.class, "colorSens3");
    }

    public void setNormalizedReading(){
        double startTime = opMode.getRuntime();
        while ( opMode.getRuntime()- startTime < 1){
            normalizedReading1[0] = (normalizedReading1[0] + colorSens1.red())/2;
            normalizedReading1[1] = (normalizedReading1[1] + colorSens1.green())/2;
            normalizedReading1[2] = (normalizedReading1[2] + colorSens1.blue())/2;

            normalizedReading2[0] = (normalizedReading2[0] + colorSens2.red())/2;
            normalizedReading2[1] = (normalizedReading2[1] + colorSens2.green())/2;
            normalizedReading2[2] = (normalizedReading2[2] + colorSens2.blue())/2;
        }
    }
    public void runWheels(int direction){
        if(direction == 1){
            intakeServo1.setPosition(1);
            wheelsRunning = true;
        }else if(direction == -1){
            intakeServo1.setPosition(1);
            wheelsRunning = true;
        }else{
            intakeServo1.setPosition(0);
            wheelsRunning = false;
        }
    }

    public int getColor(){
        int[] diffcolor1 = new int[]{colorSens1.red() - normalizedReading1[0], colorSens1.green() - normalizedReading1[1],colorSens1.blue() - normalizedReading1[2]};
        int[] diffcolor2 = new int[]{colorSens2.red() - normalizedReading2[0], colorSens2.green() - normalizedReading2[1],colorSens2.blue() - normalizedReading2[2]};

        if(diffcolor1[0] > 10 && diffcolor1[2] > 10 && diffcolor2[0] > 10 && diffcolor2[2] > 10){
            return 1;
        }else if(diffcolor1[1] >40&& diffcolor2[1] > 40){
            return 2;
        }else{
            return 0;
        }
    }

    public void intakeBall(boolean intakeMotifColors){
        indexer.updateSlots();
        if(intakeMotifColors){
            int[] allowedColors = getNeededMotifColors();
            int index = indexer.getShortestDisttoIntake();
            if(index >= 0) {
                if (indexer.getIndexerAtIntake(index)) {
                    int ballcolor = getBallColorAtSlot();
                    if ((ballcolor == 1 && allowedColors[1] > 0) || (ballcolor == 2 && allowedColors[0] > 0)) {
                        if (isBallInSlot()) {
                            runWheels(1);
                            indexer.updateSlotColor(index, ballcolor);
                            ballColoratIntake = 0;
                        } else {
                            runWheels(1);
                        }
                    } else if((ballcolor == 1 && allowedColors[1] == 0) || (ballcolor == 2 && allowedColors[0] == 0)){
                        runWheels(-1);
                    }else {
                        runWheels(1);
                    }
                } else {
                    runWheels(0);
                    indexer.moveIndexer(index, true);
                }
            }else{
                runWheels(-1);
            }

        }else{
            int index = indexer.getShortestDisttoIntake();
            ballColoratIntake = getColor();
            if(index >= 0) {
                if (indexer.getIndexerAtIntake(index)) {
                    if (isBallInSlot()) {
                        int ballcolor = getBallColorAtSlot();
                        runWheels(1);
                        indexer.updateSlotColor(index, ballcolor);
                        ballColoratIntake = 0;
                    } else {
                        runWheels(1);
                    }
                } else {
                    runWheels(1);
                    indexer.moveIndexer(index, true);
                }
            }else {
                runWheels(-1);
            }

        }

    }

    public boolean isBallInSlot(){
        return (300 <= (colorSens3.blue() + colorSens3.green() + colorSens3.red()));
    }

    public int checkBalColor(){
       return getColor();
    }

    public int[] getNeededMotifColors(){
        int purpleAmount = (2 - indexer.checkHasColor(1));
        int greenAmount = (1 - indexer.checkHasColor(2));
        return new int[] {greenAmount, purpleAmount};
    }

    public int getBallColorAtSlot(){
        return ballColoratIntake;
    }
}
