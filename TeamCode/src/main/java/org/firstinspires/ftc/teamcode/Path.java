package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutonomousDrive;

import java.util.ArrayList;

public class Path{


    AutonomousDrive ad;

    ArrayList<double[]> path = new ArrayList<>();
    LinearOpMode opMode;

    private DistanceUnit unit = DistanceUnit.INCH;
    private AngleUnit angleUnit = AngleUnit.DEGREES;

    double[] startPose;



    public Path(AutonomousDrive ad, LinearOpMode opMode, double[] startPose, double startTangent){
        this.ad = ad;
        this.startPose = startPose;

        this.path.add(startPose);

        double[] controlPoint1 = makeControlPoint(startPose, startTangent);

        this.path.add(controlPoint1);
        this.opMode = opMode;
    }

    public void setUnit(DistanceUnit unit){
        this.unit = unit;
    }

    public int getLength(){
        return path.size();
    }


    public void addSpline( double x, double y,double heading, double endTangent){
        double[] point2 = new double[]{x,y,heading};

        double[] controlPoint2 = makeControlPoint(point2, endTangent);
        double[] controlPoint3 = makeControlPoint(point2, endTangent + 180);

        path.add(controlPoint2);
        path.add(point2);
        path.add(controlPoint3);

    }



    public boolean isComplete(){

        double targetXDist = ad.getX()-path.get(path.size()-1)[0];
        double targetYDist = ad.getY()-path.get(path.size()-1)[1];

        return !(Math.abs(targetXDist) >ad.POS_ERROR_TOLERANCE2 ||  Math.abs(targetYDist) > ad.POS_ERROR_TOLERANCE2);

    }
    private double[] pathFunc(double t, int index){
        index *= 3;
        index = Math.min(index, path.size() - 5);
        double[] point = splineFunc(path.get(index), path.get(index+1), path.get(index+2), path.get(index+3),t);
        return point;
    }

    private double[] splineFunc(double[] point1, double[] point2,double[] point3,double[] point4,double t) {


        double angle1 = point1[2];
        double angle2 = point2[2];

        double[] p1 =  blend(point1, point2, t);
        double[] p2 =  blend(point2, point3, t);
        double[] p3 =  blend(point3, point4, t);
        double[] p4 =  blend(p1, p2, t);
        double[] p5 =  blend(p2, p3, t);
        double[] p6 =  blend(p4, p5, t);

        double heading = (1-t)*angle1 + t * angle2;
        p6[2] = heading;
        double[] point = p6;
        return point;


    }
    private double[] blend(double[] point1, double[] point2,double t){
        double x = (1-t)*point1[0]+t*point2[0];
        double y = (1-t)*point1[1]+t*point2[1];
        double heading = (1-t)*point1[2]+ t*point2[2];

        double[] point = new double[]{x,y,heading};
        return point;
    }

    private double[] makeControlPoint(double[] point1,double angle1){
        double x = point1[0]+ Math.sin(Math.toRadians(angle1-90));
        double y = point1[1] +  Math.cos(Math.toRadians(angle1 - 90));;

        double[] point = new double[]{x,y,point1[2]};
        return  point;

    }
    public double[] getPoint(double tick){
        int index = (int)(Math.floor(tick));
        double t = tick - index;
        return pathFunc(t,index);
    }



}
