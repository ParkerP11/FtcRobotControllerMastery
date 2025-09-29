package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArtTracker {
    int[][] artPoses = new int[3][2];

    int[] motif = new int[3];

    private LinearOpMode opMode;

    public ArtTracker(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void setMotif(int[] newMotif){
        motif = newMotif;
    }

    public int[] getMotif(){
        return motif;
    }

    public int getTargetArt() {
        int[] missingArt = getMissingArt();
        if(missingArt[0] > 0){
            return 1;
        }else if(missingArt[1] > 0){
            return 2;
        }
        return -1;
    }

    public int[] getMissingArt(){
        int green = 0;
        int purple = 0;
        for (int i = 0; i < motif.length; i++) {
            if (artPoses[i][0] == 2) {
                green++;
            } else if (artPoses[i][0] == 1) {
                purple++;
            }
        }

        if(2 - purple >= 2 && 1-green >= 0) {
            return new int[]{2 - purple, 1 - green};
        }else{
            return new int[]{2, 1};
        }

    }
}
