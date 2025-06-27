package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp
public class AutoPipelineDetection extends OpMode {

    VisionPortal portal;
    ArrayList<ColorBlobLocatorProcessor> useProcessors;

    SingleFrameImageProcessing pipeline;
    OpenCvWebcam camera;
    AutoStructuringElement autoStructuringElement;

    Servo led;
    Servo servo;

    private final int exposure = 7; //15 (actual value) - 20
    private final int gain = 2;

    //0.32
    private final double LED_BRIGHTNESS = 1;


    //Sample bestSample = FindBestSample.findBestSample();

    public void init() {

        /**
        ColorBlobLocatorProcessor blueColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(org.firstinspires.ftc.vision.opencv.ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 1.0, 1.0, -1.0))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(7)                               // Smooth the transitions between different colors in image
                .build();

        ColorBlobLocatorProcessor yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(org.firstinspires.ftc.vision.opencv.ColorRange.YELLOW)         // use a predefined color match
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

        portal = new VisionPortal.Builder()
                .addProcessors(blueColorLocator, yellowColorLocator, redColorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        useProcessors = new ArrayList<>(Arrays.asList(blueColorLocator, yellowColorLocator));
         */

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        led = hardwareMap.get(Servo.class, "LED");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        pipeline = new SingleFrameImageProcessing();

        servo = hardwareMap.get(Servo.class, "Servo");

        camera =  OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(pipeline.CAMERA_WIDTH, pipeline.CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN, OpenCvWebcam.StreamFormat.MJPEG);
                camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
                camera.getExposureControl().setExposure(exposure, TimeUnit.MILLISECONDS);
                camera.getGainControl().setGain(gain);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        autoStructuringElement = new AutoStructuringElement(led, camera);
        autoStructuringElement.initialize();

    }

    public void loop() {

        if (gamepad1.dpad_left) {
            autoStructuringElement.loop();
        }

        if (pipeline.isDoneCapturing()) {
            servo.setPosition(0.525 + (pipeline.angles / 300));
        }
    }
}