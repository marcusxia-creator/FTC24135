package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.CamFieldProfile;
import org.opencv.core.Scalar;

@Config
public class VisionConfigs {

    //Coarse Vision Camera Parameters
    public static CamFieldProfile Arducam               = new CamFieldProfile(-70,-52.5,320,240);
    public static Pose3D CamPos                         = new Pose3D(new Position(DistanceUnit.CM,0.0,4.0,12.0,0), new YawPitchRollAngles(AngleUnit.DEGREES,0,50,0,0));

    public final static int EXPOSURE = 7;
    public final static int GAIN = 2;
    public final static int LED_BRIGHTNESS = 1;

    public final static int MAX_FRAMES = 3;

    public static Scalar RANGE_HIGH; // = new Scalar(50, 255, 255);
    public static Scalar RANGE_LOW;  // = new Scalar(/** 20 */ 15, 100, 100);

    public static final int CAMERA_WIDTH = 320;
    public static final int CAMERA_HEIGHT = 240;

    //Originally 40000
    public static double maxArea = CAMERA_HEIGHT * CAMERA_HEIGHT * VisionConfigs.sample_Proportion;
    //Originally 6500
    public static double minArea = maxArea/1.5;

    //The percentage of sample detected by the camera
    public static double sample_Proportion = 0.4;
}
