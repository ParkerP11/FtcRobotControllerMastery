package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutonomousDrive;

import java.util.ArrayList;

public class Path{


    AutonomousDrive ad;

    ArrayList<Pose2D> path = new ArrayList<>();
    LinearOpMode opMode;

    private DistanceUnit unit = DistanceUnit.INCH;
    private AngleUnit angleUnit = AngleUnit.DEGREES;

    Pose2D startPose;



    public Path(AutonomousDrive ad, LinearOpMode opMode, Pose2D startPose, double startTangent){
        this.ad = ad;
        this.startPose = startPose;

        this.path.add(startPose);

        Pose2D controlPoint1 = makeControlPoint(startPose, startTangent);

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
        Pose2D point2 = new Pose2D(unit, x,y,AngleUnit.DEGREES,heading);

        Pose2D controlPoint2 = makeControlPoint(point2, endTangent);

        path.add(point2);
        path.add(controlPoint2);

    }



    public boolean isComplete(){

        double targetXDist = ad.getX()-path.get(path.size()-1).getX(DistanceUnit.INCH);
        double targetYDist = ad.getY()-path.get(path.size()-1).getY(DistanceUnit.INCH);

        return !(Math.abs(targetXDist) >ad.POS_ERROR_TOLERANCE2 ||  Math.abs(targetYDist) > ad.POS_ERROR_TOLERANCE2);

    }
    private Pose2D pathFunc(double t){
        int x = (int)(Math.floor(t));
        Pose2D point = splineFunc(path.get(x), path.get(x+1), path.get(x+2), path.get(x+3),t);
        return point;
    }

    private Pose2D splineFunc(Pose2D point1, Pose2D point2,Pose2D point3,Pose2D point4,double t) {
        if(t % 1 == 0 && t != 0) {
            t = 1;
        }else{
            t = t % 1;
        }

        double angle1 = point1.getHeading(AngleUnit.DEGREES);
        double angle2 = point2.getHeading(AngleUnit.DEGREES);

        double x1  = blend(point1, point2, t).getX(unit);
        double y1  = blend(point1, point2, t).getY(unit);

        double x2  = blend(point2, point3, t).getX(unit);
        double y2  = blend(point2, point3, t).getY(unit);

        double x3  = blend(point3, point4, t).getX(unit);
        double y3  = blend(point3, point4, t).getX(unit);

        double x4 = (1-t)*x1 + t * x2;
        double y4 = (1-t)*y1 + t * y2;

        double x5 = (1-t)*x2 + t * x3;
        double y5 = (1-t)*y2 + t * y3;

        double x = (1-t)*x4 + t * x5;
        double y = (1-t)*y4 + t * y5;
        double heading = (1-t)*angle1 + t * angle2;
        Pose2D point = new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES,heading);
        return point;


    }
    private Pose2D blend(Pose2D point1, Pose2D point2,double t){
        t = Math.max(0,Math.min(1,t));

        double x = (1-t)*point1.getX(DistanceUnit.INCH)+t*point2.getX(DistanceUnit.INCH);
        double y = (1-t)*point1.getY(DistanceUnit.INCH)+t*point2.getY(DistanceUnit.INCH);
        double heading = (1-t)*point1.getHeading(AngleUnit.DEGREES)+ t*point2.getHeading(AngleUnit.DEGREES);

        Pose2D point = new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES,heading);
        return point;
    }

    private Pose2D makeControlPoint(Pose2D point1,double angle1){
        double x = point1.getX(DistanceUnit.INCH) + Math.sin(Math.toRadians(angle1));
        double y = point1.getY(DistanceUnit.INCH) -  Math.cos(Math.toRadians(angle1));;

        Pose2D point = new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES,point1.getHeading(AngleUnit.DEGREES));
        return  point;

    }
    public Pose2D getPoint(double tick){
        if(tick >= path.size()){
            return path.get(path.size()-1);
        }
        tick = Math.max(0,tick);
        double x = pathFunc(tick).getX(DistanceUnit.INCH);
        double y = pathFunc(tick).getY(DistanceUnit.INCH);
        Pose2D point = new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES,path.get((int)(Math.floor(tick))).getHeading(AngleUnit.DEGREES));
        return point;
    }



}
