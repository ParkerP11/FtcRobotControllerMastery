package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    Servo intakeServo1;
    ColorSensor colorSens1;
    LinearOpMode opMode;

    final int[] PURPLE = new int[]{128,0,128};
    final int[] GREEN = new int[]{0,256,0};

    final int COLOR_RANGE = 25;

    boolean wheelsRunning = false;

    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
        intakeServo1 = opMode.hardwareMap.get(Servo.class, "intakeServo1");
        colorSens1 = opMode.hardwareMap.get(ColorSensor.class, "colorSens1");
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

    public void intakeBall(int color){
        if(color != getColor() || color == -1){
            runWheels(-1);
        }else{
            runWheels(1);
        }
    }
}
