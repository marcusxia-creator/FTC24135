package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/** TRY CONVERT ALL FOOTAGE TO MAT USING bitmapToMat **/

@TeleOp
@Config
public class SampleColorMasking extends LinearOpMode {
    public static boolean MASK_TOGGLE = false;
    public static boolean CONTOUR_TOGGLE = false;
    public static Scalar RANGE_HIGH = new Scalar(200,170,50,255); //original 140,140,140,255
    public static Scalar RANGE_LOW = new Scalar(120,100,0,100); //Original 50,50,50,0

    /** Rang:
     * Yellow:
        * Range high (200,170,50,256)
        * Range low (120,100,0,100)
     */

    public static class CameraProcessor implements VisionProcessor, CameraStreamSource {
        Mat rgbMat = new Mat();

        Mat hierarchy = new Mat();

        private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));
        private final AtomicReference<Bitmap> maskedFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));
        private final AtomicReference<Bitmap> contourFrame = new AtomicReference<>(Bitmap.createBitmap(1,1, Bitmap.Config.RGB_565));

        private final List<MatOfPoint> contours = new ArrayList<>();

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }

        public Bitmap getMaskedFrameBitmap(){
            return maskedFrame.get();
        }
        public Bitmap getLastFrame(){
            return lastFrame.get();
        }
        public Bitmap getContourFrame(){
            return contourFrame.get();
        }

        @Override
        public void init(int width, int height, CameraCalibration cameraCalibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, bitmap);
            lastFrame.set(bitmap);

            Core.inRange(frame, RANGE_LOW, RANGE_HIGH, rgbMat);
            Bitmap bitMap_Mask = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgbMat, bitMap_Mask);
            maskedFrame.set(bitMap_Mask);

            /**Imgproc.findContours(rgbMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(frame, contours, -1, new Scalar(0,255,0,255), 1);
            Bitmap bitmapContour = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(frame, bitmapContour);
            contourFrame.set(bitmapContour);
             **/

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int i, int i1, float v, float v1, Object o) {

        }

    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        final SampleColorMasking.CameraProcessor processor = new SampleColorMasking.CameraProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Web_Cam"))
                .build();
        waitForStart();

        while(opModeIsActive()) {
            if(MASK_TOGGLE){
                FtcDashboard.getInstance().sendImage(processor.getMaskedFrameBitmap());
            }
            /**if(CONTOUR_TOGGLE) {
                /**Technically speaking we might be able to send the masked image into the processFrame and get the contour, testing needed **/
             /**   FtcDashboard.getInstance().sendImage(processor.getContourFrame());
            } **/
            else{
                FtcDashboard.getInstance().sendImage(processor.getLastFrame());
            }
        }
    }
}
