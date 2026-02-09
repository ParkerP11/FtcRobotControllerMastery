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
import java.util.Arrays;
import java.util.List;

import javax.lang.model.type.NullType;

public class BallTrachPipeline extends OpenCvPipeline {
    private final double GREEN_THREASHOLD = 150;
    private final double PURPLE_THREASHOLD = 100;

    private final int CONTOUR_MIN_SIZE = 50;
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

    private Mat kernal1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(3,3));

    @Override
    public Mat processFrame(Mat input) {
        contoursP.clear();
        contoursG.clear();

        Core.inRange(input, lowHSVG, highHSVG, greenMat);
        Core.inRange(input, lowHSVP, highHSVP, purpleMat);

        greenMat = fillHoles(greenMat);
        purpleMat = fillHoles(purpleMat);

        boolean hasContours = true;
        int iteration = 1;
        while (hasContours) {
            hasContours = getContours();
        }
        getBallCenters(1);
        getBallCenters(2);

        return input;
    }

    private void getBallCenters(int color){
        switch (color){
            case 1:
                if(contoursP.size() > 0){
                    for(int i = 0; i < contoursP.size(); i++){
                        Moments m = Imgproc.moments(contoursP.get(i));
                        if(m.m00 > 0) {
                            double cx = m.m10 / m.m00;
                            double cy = m.m10 / m.m00;
                            ballCentersP.add(new Point(cx, cy));
                        }
                    }
                }
                break;
            case 2:
                if(contoursG.size() > 0){
                    for(int i = 0; i < contoursG.size(); i++){
                        Moments m = Imgproc.moments(contoursG.get(i));
                        if(m.m00 > 0) {
                            double cx = m.m10 / m.m00;
                            double cy = m.m10 / m.m00;
                            ballCentersG.add(new Point(cx, cy));
                        }
                    }
                }
                break;
        }
    }

    private boolean getContours(){
        Mat distMat1 = new Mat();
        Mat distMat2 = new Mat();
        Mat peaksMat1 = new Mat();
        Mat peaksMat2 = new Mat();

        boolean addedorReplace = false;

        Imgproc.erode(greenMat, greenMat, kernal1);
        Imgproc.erode(purpleMat, purpleMat, kernal1);



        Imgproc.distanceTransform(greenMat, distMat1,Imgproc.DIST_L2,5 );
        Imgproc.distanceTransform(purpleMat, distMat2,Imgproc.DIST_L2,5 );

        Core.normalize(distMat1, distMat1, 0.0, 1.0, Core.NORM_MINMAX);
        Core.normalize(distMat2, distMat2, 0.0, 1.0, Core.NORM_MINMAX);

        Imgproc.threshold(distMat1, peaksMat1, GREEN_THREASHOLD, 1.0, Imgproc.THRESH_BINARY);
        Imgproc.threshold(distMat2, peaksMat2, PURPLE_THREASHOLD, 1.0, Imgproc.THRESH_BINARY);

        peaksMat1.convertTo(peaksMat1, CvType.CV_8U, 255);
        peaksMat2.convertTo(peaksMat2, CvType.CV_8U, 255);
        greenMat = peaksMat1.clone();
        purpleMat = peaksMat2.clone();

        List<MatOfPoint> contoursG1 = new ArrayList<>();
        List<MatOfPoint> contoursP1 = new ArrayList<>();
        Imgproc.findContours(peaksMat1, contoursG1, hierarchry, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(peaksMat2, contoursP1, hierarchry, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if(contoursG1.size() > 0){
            if(contoursG.size() > 0) {
                for (int i = 0; i < contoursG.size(); i++) {
                    for(int j = 0; j < contoursG1.size(); j++){
                        if(Imgproc.contourArea(contoursG1.get(j)) > CONTOUR_MIN_SIZE){
                            MatOfPoint cnt = contoursG1.get(j);
                            if(!areContoursOverlapping(greenMat.size(), cnt, contoursG.get(i))){
                                contoursG.add(cnt);
                                addedorReplace = true;
                            }else{
                                contoursG.set(i, cnt);
                                addedorReplace = true;
                            }
                        }
                    }
                }
            }else {
                for (int i = 0; i < contoursG1.size(); i++) {
                    if (Imgproc.contourArea(contoursG1.get(i)) > CONTOUR_MIN_SIZE) {
                        MatOfPoint cnt = contoursG1.get(i);
                        contoursG.add(cnt);
                        addedorReplace = true;
                    }
                }
            }
        }

        if(contoursP1.size() > 0){
            if(contoursP.size() > 0) {
                for (int i = 0; i < contoursP.size(); i++) {
                    for(int j = 0; j < contoursP1.size(); j++){
                        if(Imgproc.contourArea(contoursP1.get(j)) > CONTOUR_MIN_SIZE){
                            MatOfPoint cnt = contoursP1.get(j);
                            if(!areContoursOverlapping(purpleMat.size(), cnt, contoursP.get(i))){
                                contoursP.add(cnt);
                                addedorReplace = true;
                            }else{
                                contoursP.set(i, cnt);
                                addedorReplace = true;
                            }
                        }
                    }
                }
            }else {
                for (int i = 0; i < contoursP1.size(); i++) {
                    if (Imgproc.contourArea(contoursP1.get(i)) > CONTOUR_MIN_SIZE) {
                        MatOfPoint cnt = contoursP1.get(i);
                        contoursP.add(cnt);
                        addedorReplace = true;
                    }
                }
            }
        }


        return addedorReplace;
    }

    private Mat fillHoles(Mat mask){
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Find contours with hierarchy
        Imgproc.findContours(
                mask.clone(),          // clone so original mask isn't modified
                contours,
                hierarchy,
                Imgproc.RETR_CCOMP,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        // Create output mask
        Mat filled = Mat.zeros(mask.size(), mask.type());

        // If no hierarchy found, return copy of original mask
        if (hierarchy.empty()) {
            return mask.clone();
        }

        // hierarchy shape: [1, numContours, 4]
        for (int i = 0; i < contours.size(); i++) {
            double[] h = hierarchy.get(0, i);
            int parent = (int) h[3];

            // Parent == -1 → outer contour
            if (parent == -1) {
                Imgproc.drawContours(
                        filled,
                        contours,
                        i,
                        new Scalar(255),
                        Imgproc.FILLED
                );
            }
        }

        return filled;
    }


    public static boolean areContoursOverlapping(
            Size imageSize,
            MatOfPoint contour1,
            MatOfPoint contour2
    ) {
        // Create blank masks (single-channel)
        Mat mask1 = Mat.zeros((int) imageSize.height, (int) imageSize.width, CvType.CV_8UC1);
        Mat mask2 = Mat.zeros((int) imageSize.height, (int) imageSize.width, CvType.CV_8UC1);

        // Draw filled contours
        List<MatOfPoint> c1 = Arrays.asList(contour1);
        List<MatOfPoint> c2 = Arrays.asList(contour2);

        Imgproc.drawContours(mask1, c1, -1, new Scalar(1), Imgproc.FILLED);
        Imgproc.drawContours(mask2, c2, -1, new Scalar(1), Imgproc.FILLED);

        // Logical AND
        Mat intersection = new Mat();
        Core.bitwise_and(mask1, mask2, intersection);

        // Check if any overlapping pixels exist
        return Core.countNonZero(intersection) > 0;
    }


    public ArrayList<Integer> getArtDistFromBot(int color){
        ArrayList<Integer> out = new ArrayList<>();
        if(color == 1){
            if(ballCentersP.size() > 0) {
                for (int i = 0; i < ballCentersP.size(); i++) {
                    int dist = (int) (Math.hypot(ballCentersP.get(i).x - (SCREEN_WIDTH / 2.0), ballCentersP.get(i).y));
                    out.add(dist);
                }
            }
            return  out;
        }else if(color == 2){
            if(ballCentersG.size() > 0) {
                for (int i = 0; i < ballCentersG.size(); i++) {
                    int dist = (int) (Math.hypot(ballCentersG.get(i).x - (SCREEN_WIDTH / 2.0), ballCentersG.get(i).y));
                    out.add(dist);
                }
            }

            return  out;
        }
        else{

            if(ballCentersG.size() > 0) {
                for (int i = 0; i < ballCentersG.size(); i++) {
                    int dist = (int) (Math.hypot(ballCentersG.get(i).x - (SCREEN_WIDTH / 2.0), ballCentersG.get(i).y));
                    out.add(dist);
                }
            }
            if(ballCentersP.size() > 0) {
                for (int i = 0; i < ballCentersP.size(); i++) {
                    int dist = (int) (Math.hypot(ballCentersP.get(i).x - (SCREEN_WIDTH / 2.0), ballCentersP.get(i).y));
                    out.add(dist);
                }
            }
            return  out;
        }
    }
}
