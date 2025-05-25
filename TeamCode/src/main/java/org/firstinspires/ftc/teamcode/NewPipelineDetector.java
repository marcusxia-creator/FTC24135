package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.dense.block.MatrixOps_DDRB;
import org.jetbrains.annotations.Contract;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class NewPipelineDetector extends OpenCvPipeline {
    private final Mat hsvMat = new Mat();
    private final Mat threshold = new Mat();
    private final Mat masked = new Mat();
    private final Mat edged = new Mat();
    private Mat kernel = new Mat();
    private final Mat dilated = new Mat();
    private final Mat hierarchy = new Mat();

    /**public static Scalar RANGE_HIGH = new Scalar(120, 255, 255);
    public static Scalar RANGE_LOW = new Scalar(100, 100, 100); **/

    public static Scalar RANGE_HIGH = new Scalar(50, 255, 255);
    public static Scalar RANGE_LOW = new Scalar(20, 100, 100);

    public static int Max_Val = 90000;
    public static int Min_Val = 80000;

    List<MatOfPoint> contours = new ArrayList<>();

    MatOfPoint2f contour2f;
    RotatedRect rotatedRect;

    private MatOfPoint2f approxCurve;
    private double epsilon;

    public static double epsilonFactor = 0.02;

    private MatOfPoint2f points;

    Point[] vertices = new Point[4];
    MatOfPoint box;
    List<MatOfPoint> boxPoints = new ArrayList<>();

    RotatedRect goodRect = null;

    /**
     * Range:
     * Yellow
     * -Range_High (50, 255, 255)
     * -Range_Low (20, 100, 100)
     */

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, RANGE_LOW, RANGE_HIGH, threshold);
        Core.bitwise_and(input, input, masked, threshold);

        Imgproc.Canny(masked, edged, Min_Val, Max_Val, 7, false);

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(edged, dilated, kernel);

        Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_EXTERNAL, /**Imgproc.RETR_LIST**/ -1);


        /**if (contours != null) {
            Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 1);
        }*/


        for (MatOfPoint contour : contours) {
            contour2f = new MatOfPoint2f(contour.toArray());

            approxCurve = new MatOfPoint2f();
            epsilon = epsilonFactor * Imgproc.arcLength(contour2f, true);

            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            points = new MatOfPoint2f(approxCurve.toArray());
            rotatedRect = Imgproc.minAreaRect(points);

            goodRect = findGoodRect(rotatedRect);

            if (goodRect != null) {
                goodRect.points(vertices);
                box = new MatOfPoint(vertices);
                boxPoints.add(box);

                Imgproc.polylines(input, boxPoints, true, new Scalar(0, 255, 0), 1);
            }
        }

        /**
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            int minArea = 500;

            if (area > minArea) {
                contour2f = new MatOfPoint2f(contour.toArray());
            }
        }
         */

        if (box != null && contours != null && contour2f != null) {
            releaseMemory();
        }

        return input;
    }

    private void releaseMemory() {
        hsvMat.release();
        threshold.release();
        masked.release();
        edged.release();
        kernel.release();
        dilated.release();
        contours.clear();
        hierarchy.release();
        contour2f.release();
        box.release();
        boxPoints.clear();
        approxCurve.release();
        points.release();
        /**goodContour.clear();**/

    }

    private RotatedRect findGoodRect(@NonNull RotatedRect rect) {
        goodRect = null;

        double maxArea = 40000;
        double minArea = 6500;

        if (rect.size.area() > minArea && rect.size.area() < maxArea) {
            goodRect = rect;
        }

        return goodRect;
    }

    /*
    public List<MatOfPoint> findGoodContour() {
        int minArea = 600;
        goodContour = null;

        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > minArea) {
                goodContour.add(contour);
            }
        }
        return goodContour;
    }
     */
}
