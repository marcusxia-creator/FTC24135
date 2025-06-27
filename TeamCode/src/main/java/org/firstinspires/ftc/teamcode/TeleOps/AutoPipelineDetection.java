package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.ImageProcessTeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AutoPipelineDetection {

    VisionPortal portal;
    ArrayList<ColorBlobLocatorProcessor> useProcessors;

    AutoSingleFrameProcessing pipeline;
    OpenCvWebcam camera;
    AutoStructuringElement autoStructuringElement;

    Servo led;

    HardwareMap hardwareMap;

    private final int exposure; // recommended 7, default 15 (actual value) - 20
    private final int gain;// default value is 2;
    private final double LED_BRIGHTNESS; //recommended value 1 default value = 0.32

    private Pose2D samplePose; //Get the sample pose

    public double x;
    public double y;

    public AutoPipelineDetection(HardwareMap hardwareMap, int exposure, int gain, int brightness) {
        this.hardwareMap = hardwareMap;
        this.exposure = exposure;
        this.gain = gain;
        this.LED_BRIGHTNESS = brightness;
    }


    //Sample bestSample = FindBestSample.findBestSample();

    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        led = hardwareMap.get(Servo.class, "LED");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        pipeline = new AutoSingleFrameProcessing();

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

        autoStructuringElement = new AutoStructuringElement(led, camera, LED_BRIGHTNESS);
        autoStructuringElement.initialize();
    }

    public void update() {
        autoStructuringElement.loop();

        if (autoStructuringElement.done) {
            x = autoStructuringElement.getX();
            y = autoStructuringElement.getY();
        }
    }

    public double getRealX() {
        return x;
    }

    public double getRealY(){
        return y;
    }
}