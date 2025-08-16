package org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Point;

public class Sample {

    public Point ViscenterPoint;
    public Point relPos;
    public double ODistance;
    public double Orientation;

    public ColorBlobLocatorProcessor.Blob blob;

    public Sample(ColorBlobLocatorProcessor.Blob blob, Pose3D relCam, CamFieldProfile CamProfile){

        //Finding relPos using algorithm
        this.ViscenterPoint=CamProfile.PixelToAngle(blob.getBoxFit().center);

        double a=Math.toRadians(relCam.getOrientation().getPitch());
        double h=relCam.getPosition().z;

        double x=Math.toRadians(-ViscenterPoint.x);
        double y=Math.toRadians(-ViscenterPoint.y);

        this.relPos=new Point(
                h*(-(Math.tan(x)*Math.cos(a)/Math.tan(a-y))-(Math.tan(x)*Math.sin(a))),
                h*(1/Math.tan(a-y))+relCam.getPosition().y);

        //Distance (For FindBestSample)
        this.ODistance=Math.sqrt(Math.pow(this.relPos.x,2)+Math.pow(this.relPos.y,2));

        //Find Orientation
        Point[] points = blob.getContourPoints(); //Theoretically shouldn't work, check size of array with telemetry later
        double deltax;
        double deltay;
        //Use longest edge
        if(Math.sqrt(Math.pow(points[0].x-points[1].x,2)+Math.pow(points[0].y-points[1].y,2))>
                Math.sqrt(Math.pow(points[1].x-points[2].x,2)+Math.pow(points[1].y-points[2].y,2))
        ){deltax = points[0].x-points[1].x; deltay = points[0].y-points[1].y;}
        else{deltax = points[1].x-points[2].x; deltay = points[1].y-points[2].y;}

        if(deltax!=0){
            this.Orientation=Math.atan(deltay/deltax)-(Math.PI/2);
        } else {
            this.Orientation=0; //Avoid atan(90°)
        }

        this.blob = blob;
    }
}