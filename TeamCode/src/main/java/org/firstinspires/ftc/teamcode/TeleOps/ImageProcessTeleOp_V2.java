package org.firstinspires.ftc.teamcode.TeleOps;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.SingleFrameImageProcessing;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class ImageProcessTeleOp_V2 extends OpMode {

    private SingleFrameImageProcessing pipeline;
    private OpenCvWebcam camera;
    private FtcDashboard dashboard;

    private Servo led;

    private enum States {
        WAITING_TO_CAPTURE,
        CAPTURING,
        PROCESSING,
        DONE
    }

    States currentState = States.WAITING_TO_CAPTURE;
    int frameIndex;

    private final int exposure = 7; //15 (actual value) - 20
    private final int gain = 2;

    //0.32
    private final double LED_BRIGHTNESS = 1;

    @Override
    public void init () {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        led = hardwareMap.get(Servo.class, "LED");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        pipeline = new SingleFrameImageProcessing();

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

        led.setPosition(LED_BRIGHTNESS);

        telemetry.addLine("Opening_Camera...");
        telemetry.update();

        currentState = States.CAPTURING;
    }

    @Override
    public void loop () {
        telemetry.addData("State", currentState);
        led.setPosition(LED_BRIGHTNESS);

        switch (currentState) {
            case CAPTURING:
                telemetry.addData("Frame Captured", pipeline.frameBuffer.size());

                if (pipeline.isDoneCapturing()) {
                    camera.stopStreaming();
                    currentState = States.PROCESSING;
                    frameIndex = 0;
                }
                break;
            case PROCESSING:
                List<Mat> frames = pipeline.frameBuffer;

                if (frameIndex < frames.size()) {
                    Mat frame = frames.get(frameIndex);

                    Mat processed = pipeline.processSingleFrame(frame);
                    Bitmap bitmap = Bitmap.createBitmap(processed.cols(), processed.rows(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(processed, bitmap);

                    dashboard.sendImage(bitmap);
                    telemetry.addLine("Sent frame" + frameIndex + "to dashboard");

                    //sleep(1000);

                    processed.release();
                    frame.release();
                    bitmap.recycle();

                    /**
                    if (pipeline.isDoneProcessing()) {
                        telemetry.addLine("Good Rect found - Finished Capturing");
                        pipeline.clearBuffer();
                        currentState = States.DONE;
                        break;
                    }
                    */

                    frameIndex ++;
                }
                else {
                    pipeline.clearBuffer();
                    currentState = States.DONE;
                }
                break;
            case DONE:
                pipeline.releaseMemory();
                telemetry.addLine("Finished Processing All Frames");
                telemetry.addData("Angles", pipeline.angles);
                telemetry.addData("realX", pipeline.realX);
                telemetry.addData("realY", pipeline.realY);
                telemetry.addData("Frame Rate", camera.getFps());
                break;
        }



        telemetry.update();
    }
}
