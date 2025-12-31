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

    final int COLOR_RANGE = 25;

    boolean wheelsRunning = false;

    int motifNum = 0;


    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
        intakeServo1 = opMode.hardwareMap.get(Servo.class, "intakeServo1");
        colorSens1 = opMode.hardwareMap.get(ColorSensor.class, "colorSens1");
        colorSens2 = opMode.hardwareMap.get(ColorSensor.class, "colorSens2");
        colorSens3 = opMode.hardwareMap.get(ColorSensor.class, "colorSens3");
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
        int[] color = new int[]{colorSens1.red(), colorSens1.green(),colorSens1.blue()};
        if(color[1] > color[0] && color[1] > color[2]){
            if(color[1] > GREEN[1]-COLOR_RANGE){
                return 2;
            }else{
                return 0;
            }
        }else{
            if(color[0] > PURPLE[0]-COLOR_RANGE && color[2] > PURPLE[2]-COLOR_RANGE){
                return 1;
            }else{
                return 0;
            }
        }
    }

    public void intakeBall(boolean intakeMotifColors){
        indexer.updateSlots();
        if(intakeMotifColors){
            int[] allowedColors = getNeededMotifColors();
            int index = indexer.getShortestDisttoIntake();
            if(index >= 0) {
                if (indexer.getIndexerAtIntake(index)) {
                    int ballcolor = checkBalColor();
                    if (ballcolor == 1 && allowedColors[1] > 0 || ballcolor == 2 && allowedColors[0] > 0) {
                        if (isBallInSlot()) {
                            runWheels(0);
                            indexer.updateSlotColor(index, ballcolor);
                        } else {
                            runWheels(1);
                        }
                    } else {
                        runWheels(-1);
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
            if(index >= 0) {
                if (indexer.getIndexerAtIntake(index)) {
                    if (isBallInSlot()) {
                        int ballcolor = checkBalColor();
                        runWheels(0);
                        indexer.updateSlotColor(index, ballcolor);
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
        if(colorSens3.red() > PURPLE[0] && colorSens3.blue() > PURPLE[1] && colorSens3.green() > PURPLE[2]){
            return 1;
        }else if(colorSens3.red() > GREEN[0] && colorSens3.blue() > GREEN[1] && colorSens3.green() > GREEN[2]){
            return 2;
        }else{
            return 0;
        }

    }

    public int[] getNeededMotifColors(){
        int purpleAmount = (2 - indexer.checkHasColor(1));
        int greenAmount = (1 - indexer.checkHasColor(2));
        return new int[] {greenAmount, purpleAmount};
    }
}
