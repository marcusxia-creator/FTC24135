package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;

public class IceWaddlerConfig {
    //Odometry Settings
    public static double odoXOffset                                             = -149.225;
    public static double odoYOffset                                             = -165.1;
    public static GoBildaPinpointDriver.GoBildaOdometryPods odoEncoderResolution= GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static GoBildaPinpointDriver.EncoderDirection xEncoderDirection      = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection yEncoderDirection      = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static PIDController vController = new PIDController(1,0,0);
    public static PIDController rotVController = new PIDController(1,0,0);
}
