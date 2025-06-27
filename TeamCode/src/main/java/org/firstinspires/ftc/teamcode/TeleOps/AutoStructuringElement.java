package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

public class AutoStructuringElement {

    private Servo led;
    private OpenCvWebcam camera;
    private AutoSingleFrameProcessing pipeline;

    private final double LED_BRIGHTNESS;

    public AutoStructuringElement(Servo led, OpenCvWebcam camera, double LED_BRIGHTNESS) {
        this.led = led;
        this.camera = camera;
        this.LED_BRIGHTNESS = LED_BRIGHTNESS;
    }

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

    public void initialize() {
        pipeline = new AutoSingleFrameProcessing();

        led.setPosition(LED_BRIGHTNESS);
        currentState = States.CAPTURING;
    }

    public void loop () {
        led.setPosition(LED_BRIGHTNESS);

        switch (currentState) {
            case CAPTURING:

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

                    pipeline.processSingleFrame(frame);

                    frame.release();

                    frameIndex ++;
                }
                else {
                    pipeline.clearBuffer();
                    currentState = States.DONE;
                }
                break;
            case DONE:
                pipeline.releaseMemory();
                break;
        }
    }
}
