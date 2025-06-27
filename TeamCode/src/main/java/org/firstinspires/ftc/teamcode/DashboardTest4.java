package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.coarsevisionproc.CamFieldProfile;
import org.firstinspires.ftc.teamcode.coarsevisionproc.Sample;
import org.firstinspires.ftc.teamcode.coarsevisionproc.FindBestSample;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous

public class DashboardTest4 extends LinearOpMode {

    @Override
    public void runOpMode()
    {
        ColorBlobLocatorProcessor blueColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(7)                               // Smooth the transitions between different colors in image
                .build();

        ColorBlobLocatorProcessor yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(7)                               // Smooth the transitions between different colors in image
                .build();

        ColorBlobLocatorProcessor redColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(7)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessors(blueColorLocator,yellowColorLocator,redColorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Web_Cam"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        List<ColorBlobLocatorProcessor> useProcessors=new ArrayList<>(Arrays.asList(blueColorLocator,yellowColorLocator));

        CamFieldProfile camFieldProfile = new CamFieldProfile(-65.36816529,-36.76959297,320,240);
        Pose3D relCam = new Pose3D(new Position(DistanceUnit.CM,0.0,0.0,25.0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,30,0.0,0));

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit())
        {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            Sample bestSample=FindBestSample.findBestSample(useProcessors,relCam,camFieldProfile);

            if(bestSample!=null) {
                telemetry.addLine(" Area Density Aspect  Center");

                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)  (%3d,%3d)  (%3d,%3d)",
                        bestSample.blob.getContourArea(), bestSample.blob.getDensity(), bestSample.blob.getAspectRatio(), (int) bestSample.blob.getBoxFit().center.x, (int) bestSample.blob.getBoxFit().center.y, (int) bestSample.ViscenterPoint.x, (int) bestSample.ViscenterPoint.y, (int) bestSample.relPos.x, (int) bestSample.relPos.y));
            }

            FtcDashboard.getInstance().startCameraStream(portal, 0);
            telemetry.update();
            sleep(50);
        }
    }
}
