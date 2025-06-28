package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.CamFieldProfile;

public class VisionConfigs {

    //Coarse Vision Camera Parameters
    public static CamFieldProfile Arducam               = new CamFieldProfile(-70,-52.5,320,240);
    public static Pose3D CamPos                         = new Pose3D(new Position(DistanceUnit.CM,0.0,4.0,12.0,0), new YawPitchRollAngles(AngleUnit.DEGREES,0,50,0,0));

    public final static int EXPOSURE = 7;
    public final static int GAIN = 2;
    public final static int LED_BRIGHTNESS = 1;
}
