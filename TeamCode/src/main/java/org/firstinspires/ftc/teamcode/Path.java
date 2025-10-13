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
    ArrayList<Double> weights = new ArrayList<>();
    LinearOpMode opMode;

    private DistanceUnit unit = DistanceUnit.INCH;
    private AngleUnit angleUnit = AngleUnit.DEGREES;



    public Path(AutonomousDrive ad, LinearOpMode opMode, ArrayList<Pose2D> points, ArrayList<Double> weights){
        this.ad = ad;
        this.path = points;

        this.weights = weights;

        this.opMode = opMode;
    }

    public void setUnit(DistanceUnit unit){
        this.unit = unit;
    }

    public int getLength(){
        return path.size();
    }

    public void addMovement(Pose2D point) {
        path.add(point);
    }



    public boolean isComplete(){

        double targetXDist = ad.getX()-path.get(path.size()-1).getX(DistanceUnit.INCH);
        double targetYDist = ad.getY()-path.get(path.size()-1).getY(DistanceUnit.INCH);

        return !(Math.abs(targetXDist) >ad.POS_ERROR_TOLERANCE2 ||  Math.abs(targetYDist) > ad.POS_ERROR_TOLERANCE2);

    }
    private Pose2D pathFunc(double t){
        int x = (int)(Math.floor(t));
        return splineFunc(path.get(x),path.get(x+1),weights.get(x), weights.get(x+1),t);
    }

    private Pose2D splineFunc(Pose2D point1, Pose2D point2, double weight1, double weight2,double t) {




        double angle1 = point1.getHeading(AngleUnit.DEGREES);
        double angle2 = point2.getHeading(AngleUnit.DEGREES);

        double x1  = (1-t)*point1.getX(DistanceUnit.INCH)+t*makeControlPoint(point1,angle1, weight1).getX(DistanceUnit.INCH);
        double y1  = (1-t)*point1.getY(DistanceUnit.INCH)+t*makeControlPoint(point1,angle1, weight1).getY(DistanceUnit.INCH);

        double x2  = (1-t)*makeControlPoint(point2,angle2+180, weight2).getX(DistanceUnit.INCH)+t*point2.getX(DistanceUnit.INCH);
        double y2  = (1-t)*makeControlPoint(point2,angle2+180, weight2).getY(DistanceUnit.INCH)+t*point2.getY(DistanceUnit.INCH);

        double x3  = (1-t)*makeControlPoint(point1,angle1, weight1).getX(DistanceUnit.INCH)+t*makeControlPoint(point2,angle2, weight2).getX(DistanceUnit.INCH);
        double y3  = (1-t)*makeControlPoint(point1,angle1, weight1).getY(DistanceUnit.INCH)+t*makeControlPoint(point2,angle2, weight2).getY(DistanceUnit.INCH);

        double x4 = (1-t)*x1 + t * x3;
        double y4 = (1-t)*y1 + t * y3;

        double x5 = (1-t)*x3 + t * x2;
        double y5 = (1-t)*y3 + t * y2;

        double x = (1-t)*x4 + t * x5;
        double y = (1-t)*y4 + t * y5;
        double heading = (1-t)*angle1 + t * angle2;
        heading = Math.max(0,Math.min(1,t));
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

    private Pose2D makeControlPoint(Pose2D point1,double angle1, double weight1){
        double x = point1.getX(DistanceUnit.INCH) + weight1 * Math.cos(Math.toRadians(angle1));
        double y = point1.getY(DistanceUnit.INCH) + weight1 * Math.sin(Math.toRadians(angle1));;

        Pose2D point = new Pose2D(DistanceUnit.INCH,x,y,AngleUnit.DEGREES,angle1);

        return point;
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
