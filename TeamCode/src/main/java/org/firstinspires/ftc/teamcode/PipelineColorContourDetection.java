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
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
        private final Mat blurred = new Mat();
        private final Mat edged = new Mat();
        private Mat kernel = new Mat();
        private final Mat dilated = new Mat();
        private final Mat hierarchy = new Mat();

        public static Scalar RANGE_HIGH = new Scalar(100, 255, 255);
        public static Scalar RANGE_LOW = new Scalar(20, 100, 100);

        /**
         Range:
            * Yellow
                -Range_High (30, 255, 255)
                -Range_Low (20, 100, 100)
         */

        public static int Min_Val = 200;
        public static int Max_Val = 700;

        List<MatOfPoint> contours = new ArrayList<>();

        private MatOfPoint2f contour2f;
        private MatOfPoint2f approxCurve;
        private double epsilon;

        private MatOfPoint2f points;

        public static RotatedRect rotatedRect;

        Point[] vertices = new Point[4];
        MatOfPoint box;
        List<MatOfPoint> boxPoints = new ArrayList<>();

        public static RotatedRect goodRect;

        private double contourArea;

        /** The following is for tuning and debugging **/
        public static boolean mask_toggle = false;
        public static boolean blur_toggle = false;
        public static boolean edge_toggle = false;
        public static boolean dilate_toggle = false;
        public static boolean approxCurve_toggle = false;

        @Override
        public Mat processFrame(@NonNull Mat inputFrame) {

            Imgproc.cvtColor(inputFrame, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, RANGE_LOW, RANGE_HIGH, threshold);

            Imgproc.GaussianBlur(threshold, blurred, new Size(3, 3), 0);

            Imgproc.Canny(blurred, edged, Min_Val, Max_Val, 3, true);

            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.dilate(edged, dilated, kernel);

            Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            //goodContour = findGoodContour(contours);


            //if (goodContour != null) {
                /** try index of 0 later **/
                //Imgproc.drawContours(inputFrame, contours, /**contours.indexOf(goodContour)**/ -1, new Scalar(0, 255, 0), 1);
            //}

            for (MatOfPoint contour : contours) {
                contour2f = new MatOfPoint2f(contour.toArray());
                approxCurve = new MatOfPoint2f();
                epsilon = 0.02 * Imgproc.arcLength(contour2f, true);

                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                points = new MatOfPoint2f(approxCurve.toArray());
                //Imgproc.drawContours(inputFrame, List.of(points), -1, new Scalar(0, 255, 0), 2);

                //Rect rect = Imgproc.boundingRect(points);
                //Imgproc.rectangle(inputFrame, rect, );

                rotatedRect = Imgproc.minAreaRect(points);
                goodRect = findGoodRect(rotatedRect);

                if (goodRect != null) {
                Imgproc.circle(inputFrame, goodRect.center, 5, new Scalar(255, 0, 0), 1);

                goodRect.points(vertices);
                box = new MatOfPoint(vertices);
                boxPoints.add(box);

                Imgproc.circle(inputFrame, vertices[0], 5, new Scalar(0, 0, 255), -1);
                Imgproc.circle(inputFrame, vertices[1], 5, new Scalar(0, 0, 255), -1);
                Imgproc.circle(inputFrame, vertices[2], 5, new Scalar(0, 0, 255), -1);
                Imgproc.circle(inputFrame, vertices[3], 5, new Scalar(0, 0, 255), -1);

                Imgproc.polylines(inputFrame, boxPoints, true, new Scalar(0, 255, 0), 1);
                /**Try the chatGPT image later**/
                }
            }

            if (contours != null && contour2f != null) {
                releaseMemory();
            }

            if (mask_toggle) {
                return threshold;
            }
            if (blur_toggle) {
                return blurred;
            }
            if (edge_toggle) {
                return edged;
            }
            if (dilate_toggle) {
                return dilated;
            }
            if (approxCurve_toggle) {
                return approxCurve;
            }
            else {
                return inputFrame;
            }
        }

        public void releaseMemory() {
            hsvMat.release();
            threshold.release();
            blurred.release();
            kernel.release();
            edged.release();
            dilated.release();
            contours.clear();
            contour2f.release();
            approxCurve.release();
            points.release();
            box.release();
            boxPoints.clear();
        }

        //private MatOfPoint findGoodContour(@NonNull List<MatOfPoint> contours, RotatedRect rect) {
        //    double maxArea = 40000;
        //    double minArea = 7000;
        //    MatOfPoint goodContour = null;

        //    for (MatOfPoint contour : contours) {
        //        double area = rect.size.area();
        //        if (area < maxArea && area > minArea) {
        //            goodContour = contour;
        //        }
        //    }

        //    return goodContour;
        //}

        private RotatedRect findGoodRect (@NonNull RotatedRect rect) {
            double maxArea = 40000;
            double minArea = 8500;

            //Integer[] width = {125, 90};
            //Integer[] height = {295, 73};
            RotatedRect goodRect = null;

            if (rect.size.area() < maxArea && rect.size.area() > minArea) {
                goodRect = rect;
            }

            return goodRect;
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
            if (pipeline.rotatedRect != null) {
                telemetry.addData("Sample angle", pipeline.rotatedRect.angle);
                telemetry.addData("Sample center", pipeline.rotatedRect.center);
                telemetry.addData("Sample size", pipeline.rotatedRect.size);
                telemetry.update();
            }
            //telemetry.addData("Contour index", pipeline.contours.indexOf(pipeline.goodContour));
        }
        camera.stopStreaming();
    }
}