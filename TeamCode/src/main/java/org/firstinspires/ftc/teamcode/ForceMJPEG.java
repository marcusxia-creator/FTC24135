package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

@TeleOp
public class ForceMJPEG extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Web_Cam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Create OpenCV camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Open the camera and try different settings
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Status", "Camera Opened");

                // Step 1: Try high resolution (forces MJPEG on many cameras)
                webcam.startStreaming(1280, 800, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Streaming", "Trying 1280x720");

                // Step 2: Delay to let the camera adjust
                sleep(2000);

                // Step 3: Restart stream at a lower resolution
                webcam.stopStreaming();
                sleep(500);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Streaming", "Switched to 640x480");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to open!");
            }
        });

        telemetry.update();
        waitForStart();
    }
}