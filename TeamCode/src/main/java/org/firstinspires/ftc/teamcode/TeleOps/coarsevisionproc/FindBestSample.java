package org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FindBestSample {

    public static ColorBlobLocatorProcessor initProcessor (org.firstinspires.ftc.vision.opencv.ColorRange colorRange){
        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)              // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(7)                               // Smooth the transitions between different colors in image
                .build();
    }


    public static Sample findBestSample(List<ColorBlobLocatorProcessor> colorLocators, Pose3D relcam, CamFieldProfile CamProfile){

        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

        for(ColorBlobLocatorProcessor colorLocator:colorLocators){
            blobs.addAll(colorLocator.getBlobs());
        }

        ColorBlobLocatorProcessor.Util.filterByArea(100, 20000, blobs);

        Sample ClosestSample;

        if(blobs.isEmpty()) {
            ClosestSample = null;
        }

        else{
            ClosestSample=new Sample(blobs.get(0),relcam,CamProfile);

            for(ColorBlobLocatorProcessor.Blob b : blobs) {
                if (new Sample(b,relcam,CamProfile).ODistance<ClosestSample.ODistance && b.getContourArea() < RobotActionConfig.SampleMaxSize){ClosestSample=new Sample(b,relcam,CamProfile);}
            }
        }


        return ClosestSample;
    }
}
