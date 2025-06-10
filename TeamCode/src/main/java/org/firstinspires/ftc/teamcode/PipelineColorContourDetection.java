package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
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
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class PipelineColorContourDetection extends LinearOpMode {
    //private static final String VUFORIA_KEY = "YOUR-VUFORIA-KEY-HERE";

    private CameraProcessor pipeline;
    private OpenCvWebcam camera;

    //private OpenCVProcessorAdapter openCVProcessorAdapter;

    public Servo servo;

    public static int exposure = -6;
    public static int gain = 30;


    @Config
    public static class CameraProcessor extends OpenCvPipeline {
        private final Mat hsvMat = new Mat();
        private final Mat threshold = new Mat();
        private final Mat blurred = new Mat();
        private final Mat edged = new Mat();
        private Mat kernel = new Mat();
        private final Mat dilated = new Mat();
        private final Mat hierarchy = new Mat();

        public static Scalar RANGE_HIGH = new Scalar(50, 255, 255);
        public static Scalar RANGE_LOW = new Scalar(20, 100, 100);

        /**
         * Range:
         * Yellow
         * -Range_High (50, 255, 255)
         * -Range_Low (20, 100, 100)
         */

        public static int Min_Val = 200;
        public static int Max_Val = 700;

        List<MatOfPoint> contours = new ArrayList<>();

        private MatOfPoint2f contour2f;
        private MatOfPoint2f approxCurve;
        private double epsilon;

        public static double epsilonFactor = 0.02;

        private MatOfPoint2f points;

        public static RotatedRect rotatedRect;

        Point[] vertices = new Point[4];
        MatOfPoint box;
        List<MatOfPoint> boxPoints = new ArrayList<>();

        public static RotatedRect goodRect;

        private double contourArea;

        public int servoRotation;

        @Override
        public Mat processFrame(@NonNull Mat inputFrame) {

            Imgproc.circle(inputFrame, new Point(180, 120), 5, new Scalar(0, 100, 200), 2);

            Imgproc.cvtColor(inputFrame, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, RANGE_LOW, RANGE_HIGH, threshold);

            Imgproc.GaussianBlur(threshold, blurred, new Size(3, 3), 0);

                                                                        /** Can try true later **/
            Imgproc.Canny(blurred, edged, Min_Val, Max_Val, 7, false);

            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
            Imgproc.dilate(edged, dilated, kernel);

                                                                /** orginially .RETR_TREE **/
            Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            //goodContour = findGoodContour(contours);


            //if (goodContour != null) {
            /** try index of 0 later **/
            //Imgproc.drawContours(inputFrame, contours, /**contours.indexOf(goodContour)**/ -1, new Scalar(0, 255, 0), 1);
            //}



            for (MatOfPoint contour : contours) {
                contour2f = new MatOfPoint2f(contour.toArray());
                approxCurve = new MatOfPoint2f();
                epsilon = epsilonFactor * Imgproc.arcLength(contour2f, true);

                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                points = new MatOfPoint2f(approxCurve.toArray());
                //Imgproc.drawContours(inputFrame, List.of(points), -1, new Scalar(0, 255, 0), 2);

                //Rect rect = Imgproc.boundingRect(points);
                //Imgproc.rectangle(inputFrame, rect, );

                rotatedRect = Imgproc.minAreaRect(points);
                goodRect = findGoodRect(rotatedRect);

                if (goodRect != null) {
                    Imgproc.circle(inputFrame, goodRect.center, 5, new Scalar(200, 0, 200), 1);

                    goodRect.points(vertices);
                    box = new MatOfPoint(vertices);
                    boxPoints.add(box);

                    Imgproc.circle(inputFrame, vertices[0], 5, new Scalar(200, 100, 100), -1);
                    Imgproc.putText(inputFrame, "Point One", vertices[0], Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(200, 100, 100), 2);
                    Imgproc.circle(inputFrame, vertices[1], 5, new Scalar(200, 100, 100), -1);
                    Imgproc.putText(inputFrame, "Point Two", vertices[1], Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(200, 100, 100), 2);
                    Imgproc.circle(inputFrame, vertices[2], 5, new Scalar(200, 100, 100), -1);
                    Imgproc.putText(inputFrame, "Point Three", vertices[2], Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(200, 100, 100), 2);
                    Imgproc.circle(inputFrame, vertices[3], 5, new Scalar(200, 100, 100), -1);
                    Imgproc.putText(inputFrame, "Point Four", vertices[3], Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(200, 100, 100), 2);

                    //Imgproc.line(inputFrame, new Point(140, -120), new Point(180, -120), new Scalar(200, 200, 0), 2);
                    //Imgproc.line(inputFrame, new Point(160, -100), new Point(160, -140), new Scalar(200, 200, 0), 2);

                    Imgproc.polylines(inputFrame, boxPoints, true, new Scalar(0, 255, 0), 1);
                    /**Try the chatGPT image later**/
                }
            }

            if (box != null && contours != null && contour2f != null) {
                releaseMemory();
            }

            /** sleep for the ammount we want **/

            return inputFrame;
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

        private RotatedRect findGoodRect(@NonNull RotatedRect rect) {
            double maxArea = 40000;
            double minArea = 6500;

            //int minWidth = 95;
            //int minHeight = 73;
            RotatedRect goodRect = null;

            double area = rect.size.area();

            /*
            List<RotatedRect> rectList = new ArrayList<>();
            List<Double> areaList = new ArrayList<>();
             */

            // Use PriorityQueue (max heap) to find the largest RotatedRect
            PriorityQueue<RotatedRect> maxHeap = new PriorityQueue<>((r1, r2) -> {
                Point[] points1 = new Point[4];
                Point[] points2 = new Point[4];

                r1.points(points1);
                r2.points(points2);

                double area1 = Imgproc.contourArea(new MatOfPoint(points1));
                double area2 = Imgproc.contourArea(new MatOfPoint(points2));

                return Double.compare(area2, area1); // Max heap
            });

            if ((area < maxArea) && (area > minArea)) {
                //goodRect = rect;
                //rectList.add(rect);
                //areaList.add(area);
                maxHeap.add(rect);
                goodRect = maxHeap.poll();
            }

            /*
            if (rectList.size() > 1) {
                List<Integer> sortedIndices = new ArrayList<>();
                for (int i = 0; i < areaList.size(); i++) {
                    sortedIndices.add(i);
                }

                // Sort indices by area in descending order
                sortedIndices.sort((i1, i2) -> Double.compare(areaList.get(i2), areaList.get(i1)));

                List<RotatedRect> filteredRects = new ArrayList<>();
                filteredRects.add(rectList.get(sortedIndices.get(0))); // Largest



                goodRect = filteredRects.get(0);
            }
             */

            return goodRect;
        }
    }

    @Override
    public void runOpMode() {

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Web_Cam");

        servo = hardwareMap.get(Servo.class, "Intake_Rotation_Servo");
        servo.setPosition(0.5);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        /**
        pipeline = new CameraProcessor();
        openCVProcessorAdapter = new OpenCVProcessorAdapter(pipeline);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new android.util.Size(320, 240))
                .addProcessor(openCVProcessorAdapter)
                .enableLiveView(true)
                .build();

        ExposureControl exposureControl;
        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode)
        exposureControl.setExposure(30,);

         **/

        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new CameraProcessor();
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

        //camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        long currentTime = 0;

        waitForStart();

        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(camera, 100);

            if (pipeline.rotatedRect != null) {
                telemetry.addData("Sample angle", pipeline.rotatedRect.angle);
                //telemetry.addData("Sample center", pipeline.rotatedRect.center);
                //telemetry.addData("Sample size", pipeline.rotatedRect.size);
                //telemetry.addData("Good Rect", pipeline.findGoodRect(pipeline.rotatedRect));
                telemetry.addData("Frame rate", camera.getFps());

                if (currentTime == 0){
                    currentTime = System.currentTimeMillis();
                }
                if (System.currentTimeMillis() - currentTime > 2000) {
                    if (pipeline.rotatedRect.angle > 45) {
                        servo.setPosition(0.65);
                    } else if (pipeline.rotatedRect.angle > 75) {
                        servo.setPosition(0.8);
                    } else if (pipeline.rotatedRect.angle < -45) {
                        servo.setPosition(0.35);
                    } else if (pipeline.rotatedRect.angle < -75) {
                        servo.setPosition(0.2);
                    } else {
                        servo.setPosition(0.5);
                    }
                    currentTime = 0;
                }

                //telemetry.addData("Vertices", pipeline.vertices[0].toString(), pipeline.vertices[1].toString(), pipeline.vertices[2].toString(), pipeline.vertices[3].toString());
                telemetry.update();
            }
            else {
                currentTime = 0;

            }



            //telemetry.addData("Contour index", pipeline.contours.indexOf(pipeline.goodContour));
        }

        /**
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera((CameraName) camera)
                .build();
         **/

        camera.stopStreaming();
    }

    /*
    public static class OpenCVProcessorAdapter implements VisionProcessor {
        private final OpenCvPipeline pipeline;

        public OpenCVProcessorAdapter(OpenCvPipeline pipeline) {
            this.pipeline = pipeline;
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            // Initialize the OpenCV pipeline
            // pipeline.init(null); // OpenFTC’s pipeline requires an OpenCvCamera, but we don’t need it
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            return pipeline.processFrame(frame); // Delegate frame processing to OpenFTC pipeline
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        }
    }
     */
}