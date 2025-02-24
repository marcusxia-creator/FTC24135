package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class PipelineColorContourDetection extends LinearOpMode {
    private OpenCvCamera camera;
    private CameraProcessor pipeline;

    @Config
    public static class CameraProcessor extends OpenCvPipeline {
        private final Mat hsvMat = new Mat();
        private final Mat threshold = new Mat();
        private final Mat hierarchy = new Mat();

        public static Scalar RANGE_HIGH = new Scalar(30, 255, 255);
        public static Scalar RANGE_LOW = new Scalar(20, 100, 100);

        List<MatOfPoint> contours = new ArrayList<>();

        public static MatOfPoint goodContour;

        @Override
        public Mat processFrame(@NonNull Mat inputFrame) {
            Imgproc.cvtColor(inputFrame, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, RANGE_LOW, RANGE_HIGH, threshold);

            Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            goodContour = findGoodContour(contours);

            if (goodContour != null) {
                /** try index of 0 later **/
                Imgproc.drawContours(inputFrame, contours, contours.indexOf(goodContour) /** -1 **/, new Scalar(0, 255, 0), 1);
            }

            return inputFrame;
        }

        private MatOfPoint findGoodContour(@NonNull List<MatOfPoint> contours) {
            double maxArea = 40000;
            double minArea = 8000;
            MatOfPoint goodContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < maxArea && area > minArea) {
                    goodContour = contour;
                }
            }

            return goodContour;
        }
    }


    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Web_Cam"), cameraMonitorViewId);
        pipeline = new CameraProcessor();
        camera.setPipeline(pipeline);
        camera.openCameraDevice();
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(camera, 30);
            //telemetry.addData("Contour index", pipeline.contours.indexOf(pipeline.goodContour));
            telemetry.update();
        }
        camera.stopStreaming();
    }
}