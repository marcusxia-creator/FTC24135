package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.FindBestSample;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.Sample;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name = "Colour Range Tuning", group = "org.firstinspires.ftc.teamcode") @Config
public class ColourRangeTuner extends OpMode {
    
    private RobotHardware robot;
    
    private VisionPortal portal;

    private Scalar MinHSV;
    private Scalar MaxHSV;
    private ColorRange colorRange;
    private ColorBlobLocatorProcessor colorBlobLocatorProcessor;

    public static double MIN_H = 0;
    public static double MIN_S = 0;
    public static double MIN_V = 0;

    public static double MAX_H = 0;
    public static double MAX_S = 0;
    public static double MAX_V = 0;

    public void init(){
        robot = new RobotHardware();
        robot.init(hardwareMap);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        MinHSV = new Scalar(MIN_H, MIN_S, MIN_V, 255.0);
        MaxHSV = new Scalar(MAX_H, MAX_S, MAX_V, 255.0);
        colorRange = new ColorRange(ColorSpace.HSV, MinHSV, MaxHSV);

        colorBlobLocatorProcessor = FindBestSample.initProcessor(colorRange);

        portal = new VisionPortal.Builder()
                .addProcessor(colorBlobLocatorProcessor)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(robot.Webcam)
                .build();

        FtcDashboard.getInstance().startCameraStream(portal, 0);
    }

    public void loop(){
        colorBlobLocatorProcessor = FindBestSample.initProcessor(colorRange);

        MinHSV = new Scalar(MIN_H, MIN_S, MIN_V, 255.0);
        MaxHSV = new Scalar(MAX_H, MAX_S, MAX_V, 255.0);
        colorRange = new ColorRange(ColorSpace.HSV, MinHSV, MaxHSV);

        colorBlobLocatorProcessor = FindBestSample.initProcessor(colorRange);

        portal = new VisionPortal.Builder()
                .addProcessor(colorBlobLocatorProcessor)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(robot.Webcam)
                .build();

        Sample bestSample = FindBestSample.findBestSample(new ArrayList<>(Arrays.asList(colorBlobLocatorProcessor)),RobotActionConfig.CamPos,RobotActionConfig.Arducam);

        if(bestSample!=null){
            telemetry.addData("Size", bestSample.blob.getContourArea());
            telemetry.addData("Detected Position", String.format("(%.2f,%.2f)", bestSample.relPos.x, bestSample.relPos.y));
        }

        telemetry.update();
    }
}


