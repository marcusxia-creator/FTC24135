package org.firstinspires.ftc.teamcode.TeleOps;

import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

public class AutoSingleFrameProcessing extends OpenCvPipeline {
    public final List<Mat> frameBuffer = new ArrayList<>();
    public final int MAX_FRAMES = 1;
    private boolean doneCapturing = false;
    public boolean isDoneCapturing() {
        return doneCapturing;
    }

    private boolean doneProcessing = false;
    public boolean isDoneProcessing() {return doneProcessing;}

    private final Mat hsvMat = new Mat();
    private final Mat masked = new Mat();
    private final Mat threshold = new Mat();
    private final Mat edged = new Mat();
    private Mat kernel = new Mat();
    private final Mat dilated = new Mat();
    private final Mat hierarchy = new Mat();

    public static Scalar RANGE_HIGH = new Scalar(50, 255, 255);
    public static Scalar RANGE_LOW = new Scalar(/** 20 */ 15, 100, 100);

    /**
     * Range:
     * Yellow
     * -Range_High (50, 255, 255)
     * -Range_Low (20 (15), 100, 100)
     * *Blue
     * -Range_High (130, 255, 255)
     * -Range_Low (100, 100, 100)
     */

    public static int Max_Val = 90000 /*700 */;
    public static int Min_Val = 80000 /*200 */;

    /**
     * edge becomes more unconnected and only those which are apparent
     * are kept with a higher value
     * Ex
     * - on a 3d sample, the surface (top) of the sample are more likely to be
     *   detected as a 2d plane without the 3d structures visible with increasing value
     * **/

    List<MatOfPoint> contours = new ArrayList<>();

    private MatOfPoint2f contour2f;
    private MatOfPoint2f approxCurve;
    private double epsilon;

    public static double epsilonFactor = 0.02;

    private MatOfPoint2f points;

    public static RotatedRect rotatedRect;

    private MatOfPoint bestContour;
    public static RotatedRect goodRect;

    public static double angles;

    //Camera Lens Constants
    public static final int CAMERA_WIDTH = 320;
    public static final int CAMERA_HEIGHT = 240;

    private final int cx = CAMERA_WIDTH / 2;
    private final int cy = CAMERA_HEIGHT / 2;

    //originally 228.5387
    private final double fx = 228.50368107873832034569896394533;
    private final double fy = 243.33592823870697717256615973888;

    //Sample Real Life Coordinates
    private final double REAL_WIDTH = 1.5;

    public double realX;
    public double realY;

    @Override
    public Mat processFrame (Mat input) {

        if (!doneCapturing) {
            // Save a clone of the input frame
            frameBuffer.add(input.clone());
            if (frameBuffer.size() >= MAX_FRAMES) {
                doneCapturing = true;
            }
        }
        return input;
    }

    public void clearBuffer() {
        for (Mat mat : frameBuffer) {
            mat.release();
        }
        frameBuffer.clear();
        doneCapturing = false;
    }

    public void processSingleFrame (Mat inputFrame) {

        angles = 0.0;
        realX = 0.0;
        realY = 0.0;

        Imgproc.cvtColor(inputFrame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, RANGE_LOW, RANGE_HIGH, threshold);

        //Imgproc.GaussianBlur(threshold, blurred, new Size(3, 3), 0);

        Core.bitwise_and(inputFrame, inputFrame, masked, threshold);

        /** Can try true later **/
        Imgproc.Canny(masked, edged, Min_Val, Max_Val, 7, false);

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(edged, dilated, kernel);

        /** orginially .RETR_TREE **/
        Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        double minDistance = Double.MAX_VALUE;
        bestContour = null;

        for (MatOfPoint contour : contours) {

            //if (contour.toArray().length < 5) continue;
            //if (Imgproc.contourArea(contour) < 100) continue;

            Moments M = Imgproc.moments(contour);
            if (M.m00 == 0) continue; // avoid division by zero

            double center_x = M.m10 / M.m00;
            double center_y = M.m01 / M.m00;

            // Distance to image center
            double dx = center_x - cx;
            double dy = center_y - cy;
            double dist = Math.sqrt(dx * dx + dy * dy);

            if (dist < minDistance) {
                minDistance = dist;
                bestContour = contour;
            }

            if (bestContour != null) {
                contour2f = new MatOfPoint2f(bestContour.toArray());
                approxCurve = new MatOfPoint2f();
                epsilon = epsilonFactor * Imgproc.arcLength(contour2f, true);

                Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

                points = new MatOfPoint2f(approxCurve.toArray());

                rotatedRect = Imgproc.minAreaRect(points);
                goodRect = findGoodRect(rotatedRect);
            }

        }

        if (goodRect != null) {
            getAngles();
            findRealCoordinate();

            doneProcessing = true;
        }

        releaseMemory();
    }

    private void getAngles() {
        double width = rotatedRect.size.width;
        double height = rotatedRect.size.height;
        double angle = goodRect.angle;
        angles = goodRect.angle;

        if (width < height) {
            angle += 90;
        }

        if (angle < 0) {
            angle += 360;
        }

        // Now angle is the direction of the **long side**
// We'll say:
        /**ORIGNINAL */
// - Angle between 0–180 → facing right
// - Angle between 180–360 → facing left
        /**Actual Value */
// - Angle between 0-90 → facing right
// - Angle between 90-180 → facing left

        if (!(angle >= 90 && angle <= 180)) {
            angles = -(90 - angles);
            //Flip the value by subtracting the right angle 90 degrees value
        }
    }

    private void findRealCoordinate() {

        // Estimated distance
        double pixelWidth = Math.min(goodRect.size.width, goodRect.size.height);
        double distance = (REAL_WIDTH * fx) / pixelWidth;

        // Offset from image center
        double dx = (goodRect.center.x - cx) / fx;
        double dy = (cy - goodRect.center.y) / fy;

        // Real-world position
        realX = dx * distance; // Left/right
        realY = dy * distance; // Up/down
    }

    public void releaseMemory() {
        if (contours != null && contour2f != null) {
            contours.clear();
            contour2f.release();
            approxCurve.release();
            points.release();
            hsvMat.release();
            masked.release();
            threshold.release();
            kernel.release();
            edged.release();
            dilated.release();
            bestContour.release();
        }
    }


    private RotatedRect findGoodRect(@NonNull RotatedRect rect) {
        double maxArea = 40000;
        double minArea = 6500;

        //int minWidth = 95;
        //int minHeight = 73;
        RotatedRect goodRect = null;

        double area = rect.size.area();


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
            maxHeap.add(rect);
            goodRect = maxHeap.poll();
        }

        return goodRect;
    }

    public double getAngle() {
        return angles;
    }

    public double getRealX() {
        return realX;
    }

    public double getRealY() {
        return realY;
    }

    public Pose2D getSamplePose() {
         return new Pose2D(realX, realY, Math.toRadians(angles));
    }
}