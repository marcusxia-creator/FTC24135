package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.atomic.AtomicReference;

public class PipelineColorContourDetection extends LinearOpMode {
    private OpenCvCamera camera;

    public static class CameraProcessor extends OpenCvPipeline {

        private final AtomicReference<Mat> lastFrame = new AtomicReference<>();
        private final AtomicReference<Mat> maskedFrame = new AtomicReference<>();
        private final AtomicReference<Mat> contourFrame = new AtomicReference<>();

        public Mat getLastFrame() {
            return lastFrame.get();
        }

        @Override
        public Mat processFrame(Mat frame) {

            return null;
        }
    }


    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final SampleColorMasking.CameraProcessor processor = new SampleColorMasking.CameraProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Web_Cam"))
                .build();
        waitForStart();
    }
}