package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallTrachPipeline extends OpenCvPipeline {
    private final double GREEN_THREASHOLD = 0.5;
    private final double PURPLE_THREASHOLD = 0.5;

    private final int SCREEN_WIDTH = 320;

    private Mat greenMat = new Mat();
    private Mat greenDistMat = new Mat();
    private Mat greenPeaks = new Mat();
    private Mat purpleMat = new Mat();
    private Mat  purpleDistMat = new Mat();
    private Mat  purplePeaks = new Mat();
    private Mat hierarchry = new Mat();
    private List<MatOfPoint> contoursG,contoursP  = new ArrayList<>();
    private List<Point> ballCentersG = new ArrayList<>();
    private List<Point> ballCentersP = new ArrayList<>();
    private final Mat kernal = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(7,7));


    private int hueminG = 0;
    private int huemaxG = 255;
    private int satminG = 0;
    private int satmaxG = 255;
    private int valminG = 0;
    private int valmaxG = 255;

    private int hueminP = 0;
    private int huemaxP = 255;
    private int satminP = 0;
    private int satmaxP = 255;
    private int valminP = 0;
    private int valmaxP = 255;

    private Scalar lowHSVG = new Scalar(hueminG, satminG, valminG);
    private Scalar highHSVG = new Scalar(huemaxG, satmaxG, valmaxG);

    private Scalar lowHSVP = new Scalar(hueminP, satminP, valminP);
    private Scalar highHSVP = new Scalar(huemaxP, satmaxP, valmaxP);


    @Override
    public Mat processFrame(Mat input) {
        contoursP.clear();
        contoursG.clear();

        Core.inRange(input, lowHSVG, highHSVG, greenMat);
        Core.inRange(input, lowHSVP, highHSVP, purpleMat);

        Imgproc.morphologyEx(greenMat,greenMat, Imgproc.MORPH_CLOSE, kernal);
        Imgproc.morphologyEx(purpleMat,purpleMat, Imgproc.MORPH_CLOSE, kernal);

        Imgproc.medianBlur(greenMat, greenMat,5);
        Imgproc.medianBlur(purpleMat, purpleMat,5);

        Imgproc.distanceTransform(greenMat, greenDistMat,Imgproc.DIST_L2,5 );
        Imgproc.distanceTransform(purpleMat, purpleDistMat,Imgproc.DIST_L2,5 );

        Core.normalize(greenDistMat, greenDistMat, 0.0, 1.0, Core.NORM_MINMAX);
        Core.normalize(purpleDistMat, purpleDistMat, 0.0, 1.0, Core.NORM_MINMAX);

        Imgproc.threshold(greenDistMat, greenPeaks, GREEN_THREASHOLD, 1.0, Imgproc.THRESH_BINARY);
        Imgproc.threshold(purpleDistMat, purplePeaks, PURPLE_THREASHOLD, 1.0, Imgproc.THRESH_BINARY);

        greenPeaks.convertTo(greenPeaks, CvType.CV_8U, 255);
        purplePeaks.convertTo(purplePeaks, CvType.CV_8U, 255);

        Imgproc.findContours(greenMat, contoursG, hierarchry, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(purpleMat, contoursP, hierarchry, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for(MatOfPoint c : contoursG){
            Moments m = Imgproc.moments(c);
            if(m.m00 > 0){
                int cx = (int)(m.m10 / m.m00);
                int cy = (int)(m.m01 / m.m00);

                ballCentersG.add(new Point(cx,cy));

                Imgproc.circle(input, new Point(cx,cy), 6, new Scalar(0,255,0), -1);
            }
        }

        for(MatOfPoint c : contoursP){
            Moments m = Imgproc.moments(c);
            if(m.m00 > 0){
                int cx = (int)(m.m10 / m.m00);
                int cy = (int)(m.m01 / m.m00);

                ballCentersP.add(new Point(cx,cy));

                Imgproc.circle(input, new Point(cx,cy), 6, new Scalar(255,0,255), -1);
            }
        }

        return input;
    }


    public ArrayList<Integer> getArtdistX(int color){
        ArrayList<Integer> out = new ArrayList<>();
        if(color == 1){
            if(ballCentersP.size() > 0) {
                for (int i = 0; i < ballCentersP.size(); i++) {
                    int xdist = (int) (ballCentersP.get(i).x - (SCREEN_WIDTH / 2.0));
                    out.add(xdist);
                }
            }
            return  out;
        }else if(color == 2){
            if(ballCentersG.size() > 0) {
                for (int i = 0; i < ballCentersG.size(); i++) {
                    int xdist = (int) (ballCentersG.get(i).x - (SCREEN_WIDTH / 2.0));
                    out.add(xdist);
                }
            }

            return  out;
        }
        else{

            if(ballCentersG.size() > 0) {
                for (int i = 0; i < ballCentersG.size(); i++) {
                    int xdist = (int) (ballCentersG.get(i).x - (SCREEN_WIDTH / 2.0));
                    out.add(xdist);
                }
            }
            if(ballCentersP.size() > 0) {
                for (int i = 0; i < ballCentersP.size(); i++) {
                    int xdist = (int) (ballCentersP.get(i).x - (SCREEN_WIDTH / 2.0));
                    out.add(xdist);
                }
            }
            return  out;
        }

    }
}
