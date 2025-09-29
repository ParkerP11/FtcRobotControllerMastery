package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class HuskyPipeline {

    HuskyLens cam1;

    LinearOpMode opMode;

    public HuskyPipeline(LinearOpMode opMode){
        this.opMode = opMode;

        cam1 = opMode.hardwareMap.get(HuskyLens.class, "cam1");

    }

    public HuskyLens.Block[] getBLocks(){
        return cam1.blocks();
    }

    public int[][] getPoses(){
        HuskyLens.Block[] blocks = getBLocks();
        if(blocks.length >  0) {
            int[][] poses = new int[blocks.length][3];
            for (int i = 0; i < blocks.length; i++){
                poses[i][0] = blocks[i].x;
                poses[i][1] = blocks[i].y;
                poses[i][2] = blocks[i].id;

            }
            return poses;
        }
        else{
            return new int[1][1];
        }
    }

    public Enum APRILTAG, COLOR, OBJECTTRACK, FACE;

    public void changeALgo(Enum algo){
        if(algo == APRILTAG){
            cam1.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }else if(algo == COLOR){
            cam1.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }else if(algo == OBJECTTRACK){
            cam1.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        }else if(algo == FACE){
            cam1.selectAlgorithm(HuskyLens.Algorithm.FACE_RECOGNITION);
        }
    }


}
