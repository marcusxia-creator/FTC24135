package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class NewTeleOp extends OpMode {
    private NewPipelineDetector pipeline;
    private OpenCvWebcam camera;

    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Web_Cam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new NewPipelineDetector();
        camera.openCameraDevice();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                //camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
                //camera.getExposureControl().setExposure(exposure, TimeUnit.MILLISECONDS);
                //camera.getGainControl().setGain(gain);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void loop() {
        FtcDashboard.getInstance().startCameraStream(camera, 100);
    }

    public void stop() {
        camera.stopStreaming();
        pipeline.releaseMemory();
    }
}
