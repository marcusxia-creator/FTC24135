package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class SingleFrameImageProcessing extends OpenCvPipeline {

    //Camera Lens Constants
    public static final int CAMERA_WIDTH = 320;
    public static final int CAMERA_HEIGHT = 240;

    private final int cx = CAMERA_WIDTH / 2;
    private final int cy = CAMERA_HEIGHT / 2;

    //originally 228.5387
    private final double fx = 228.50368107873832034569896394533;
    private final double fy = 243.33592823870697717256615973888;

    //Sample Real Life Coordinates
    private final double REAL_WIDTH = 1.5;

    public static double realX;
    public static double realY;

    @Override
    public Mat processFrame (Mat inputFrame) {

        return null;
    }
}
