package org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class FindBestSample {

    public static ColorBlobLocatorProcessor initProcessor (org.firstinspires.ftc.vision.opencv.ColorRange colorRange){

        return new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)              // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(7)                                // Smooth the transitions between different colors in image
                .build();
    }


    public static Sample findBestSample(List<ColorBlobLocatorProcessor> colorLocators, Pose3D relcam, CamFieldProfile CamProfile){
        List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

        for(ColorBlobLocatorProcessor colorLocator:colorLocators){
            blobs.addAll(colorLocator.getBlobs());
        }

        ColorBlobLocatorProcessor.Util.filterByArea(10000, 20000, blobs);

        Sample ClosestSample;

        if(blobs.isEmpty()) {
            ClosestSample = null;
        }
        else{
            ClosestSample=new Sample(blobs.get(0),relcam,CamProfile);

            for(ColorBlobLocatorProcessor.Blob b : blobs) {
                if (new Sample(b,relcam,CamProfile).ODistance<ClosestSample.ODistance){ClosestSample=new Sample(b,relcam,CamProfile);}
            }
        }

        return ClosestSample;
    }
}
