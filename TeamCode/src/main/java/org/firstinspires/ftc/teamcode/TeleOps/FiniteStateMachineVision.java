package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.FindBestSample;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.Sample;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.TimeUnit;

public class FiniteStateMachineVision {

    public enum VISIONSTATE {
        IDLE,

        VISION_COARSE_DETECT,
        VISION_COARSE_EXTEND,

        VISION_FINE_LIVE,
        VISION_FINE_STATIC,

        VISION_TURRET_GRAB,
        VISION_FINE_FAILED,
    }

    public VISIONSTATE visionState;

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private final FiniteStateMachineIntake intakeArmDrive;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private ElapsedTime visionTimer = new ElapsedTime();

    private AutoVisionProcessing autoVisionProcessing;
    public Pipeline pipeline;
    private FtcDashboard dashboard;
    private HardwareMap hardwareMap;

    private Pose2D pose2D;

    private ArrayList<ColorBlobLocatorProcessor> useProcessors;
    public  static VisionPortal portal;

    private ElapsedTime Timer = new ElapsedTime();
    public Sample bestSample;

    public boolean takeControls;

    private int i = 0;

    /**
     * Range:
     * Yellow
     * -Range_High (50, 255, 255)
     * -Range_Low (20 (15), 100, 100)
     * *Blue
     * -Range_High (130, 255, 255)
     * -Range_Low (100, 100, 100)
     */

    private final Scalar blue_Range_High = new Scalar (130, 255, 255);
    private final Scalar blue_Range_Low = new Scalar (100, 100, 100);

    private final Scalar red_Range_High = new Scalar (0.0, 0.0, 0.0);
    private final Scalar red_Range_Low = new Scalar (0.0, 0.0, 0.0);

    private final Scalar yellow_Range_High = new Scalar (50, 255, 255);
    private final Scalar yellow_Range_Low = new Scalar (/** 20 */ 15, 100, 100);

    public FiniteStateMachineVision(FtcDashboard dashboard,RobotHardware robot, HardwareMap hardwareMap, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineIntake intakeArmDrive,boolean takeControls) {
        this.dashboard = dashboard;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.intakeArmDrive = intakeArmDrive;
        this.visionState = VISIONSTATE.IDLE;
        this.hardwareMap = hardwareMap;
        this.takeControls=  takeControls ;
    }

    public void init(boolean detectBlue, boolean detectRed, boolean detectYellow){
        autoVisionProcessing = new AutoVisionProcessing(dashboard, hardwareMap);
        autoVisionProcessing.initialize();

        pipeline = new Pipeline();

        ColorBlobLocatorProcessor blueColorLocator= FindBestSample.initProcessor(org.firstinspires.ftc.vision.opencv.ColorRange.BLUE);
        ColorBlobLocatorProcessor yellowColorLocator=FindBestSample.initProcessor(org.firstinspires.ftc.vision.opencv.ColorRange.YELLOW);
        ColorBlobLocatorProcessor redColorLocator=FindBestSample.initProcessor(ColorRange.RED);

        useProcessors=new ArrayList<>();

        portal = new VisionPortal.Builder()
                .addProcessors(blueColorLocator,yellowColorLocator,redColorLocator, pipeline)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(robot.Webcam)
                .build();

        if(detectBlue)  {
            useProcessors.add(blueColorLocator);
            VisionConfigs.RANGE_HIGH = blue_Range_High;
            VisionConfigs.RANGE_LOW = blue_Range_Low;
        }
        if(detectRed)   {
            useProcessors.add(redColorLocator);
            VisionConfigs.RANGE_HIGH = red_Range_High;
            VisionConfigs.RANGE_LOW = red_Range_Low;
        }
        if(detectYellow){
            useProcessors.add(yellowColorLocator);
            VisionConfigs.RANGE_HIGH = yellow_Range_High;
            VisionConfigs.RANGE_LOW = yellow_Range_Low;
        }

    }

    public void visionLoop (boolean liveDetection) {
        switch(visionState){
            case IDLE:
                if(!takeControls){
                    Timer.reset();
                    visionState=VISIONSTATE.VISION_COARSE_DETECT;
                }
                //Change key
                if (((((gamepad_1.getButton(DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(DPAD_RIGHT) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1))

                        || (gamepad_1.getButton(DPAD_LEFT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(DPAD_LEFT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                        isButtonDebounced())) {
                    Timer.reset();
                    visionState=VISIONSTATE.VISION_COARSE_DETECT;
                }
                break;

            case VISION_COARSE_DETECT:

                robot.led.setPosition(VisionConfigs.LED_BRIGHTNESS);

                robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Coarse);
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Coarse);

                advancedIntake.runToPoint(robot, new Point(0, 0), DistanceUnit.INCH);

                intakeArmDrive.intakeState = FiniteStateMachineIntake.INTAKESTATE.INTAKE_DISABLED;

                bestSample = FindBestSample.findBestSample(useProcessors, VisionConfigs.CamPos, VisionConfigs.Arducam);
                dashboard.startCameraStream(portal, 100);

                if (bestSample != null) {
                    if (bestSample.relPos.x > -7 &
                            bestSample.relPos.x < 7 &
                            bestSample.relPos.y > 0 &
                            bestSample.relPos.y < 14
                    ) {
                        Timer.reset();
                        visionState = VISIONSTATE.VISION_COARSE_EXTEND;
                    }
                }

                break;

            case VISION_COARSE_EXTEND:
                intakeArmDrive.intakeState = FiniteStateMachineIntake.INTAKESTATE.INTAKE_EXTEND;
                intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.OPEN;

                if(bestSample!=null) {
                    if (bestSample.relPos.x > -7 &
                            bestSample.relPos.x < 7 &
                            bestSample.relPos.y > 0 &
                            bestSample.relPos.y < 14 //14
                    ) {
                        intakeArmDrive.intakeState = FiniteStateMachineIntake.INTAKESTATE.INTAKE_DISABLED;
                        advancedIntake.runToPoint(robot, bestSample.relPos, DistanceUnit.INCH);
                        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Fine);
                        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Fine);
                        robot.led.setPosition(VisionConfigs.LED_BRIGHTNESS);
                    }
                }

                if(Timer.seconds() > advancedIntake.slideExtension/27.16 - 0.1/*53544*/){
                    if (liveDetection) {
                        visionTimer.reset();
                        autoVisionProcessing.currentState = AutoVisionProcessing.States.CAPTURING;
                        visionState = VISIONSTATE.VISION_FINE_LIVE;
                    } else {
                        //Don't call for pure coarse
                        autoVisionProcessing.currentState = AutoVisionProcessing.States.CAPTURING;
                        visionState = VISIONSTATE.VISION_FINE_STATIC;
                    }
                    break;
                }

                break;

            case VISION_FINE_LIVE:

                portal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
                portal.getCameraControl(ExposureControl.class).setExposure(VisionConfigs.EXPOSURE, TimeUnit.MILLISECONDS);
                portal.getCameraControl(GainControl.class).setGain(VisionConfigs.GAIN);

                robot.led.setPosition(VisionConfigs.LED_BRIGHTNESS);

                if (visionTimer.seconds() < 0.2) {
                    autoVisionProcessing.process(true);

                    if (AutoVisionProcessing.done && autoVisionProcessing.sampleAngles != 0.0) {
                        pose2D = new Pose2D(DistanceUnit.INCH, -autoVisionProcessing.sampleX, -autoVisionProcessing.sampleY, AngleUnit.DEGREES, -autoVisionProcessing.sampleAngles);
                        AutoVisionProcessing.done = false;
                        dashboard.startCameraStream(portal, 100);
                        autoVisionProcessing.currentState = AutoVisionProcessing.States.WAITING_TO_CAPTURE;
                        autoVisionProcessing.sampleX = 0.0;
                        autoVisionProcessing.sampleY = 0.0;
                        autoVisionProcessing.sampleAngles = 0.0;
                        Timer.reset();
                        visionState = VISIONSTATE.VISION_TURRET_GRAB;
                        break;
                    }
                } else {
                    visionState = VISIONSTATE.VISION_FINE_FAILED;
                    break;
                }
                break;

            case VISION_FINE_STATIC:
                portal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
                portal.getCameraControl(ExposureControl.class).setExposure(VisionConfigs.EXPOSURE, TimeUnit.MILLISECONDS);
                portal.getCameraControl(GainControl.class).setGain(VisionConfigs.GAIN);

                robot.led.setPosition(VisionConfigs.LED_BRIGHTNESS);
                intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.OPEN;
                autoVisionProcessing.currentState = AutoVisionProcessing.States.CAPTURING;

                for (i = 0; i < VisionConfigs.MAX_FRAMES; i++) {

                    autoVisionProcessing.process(false);

                    if (AutoVisionProcessing.done && autoVisionProcessing.sampleAngles != 0.0) {
                        pose2D = new Pose2D(DistanceUnit.INCH, -autoVisionProcessing.sampleX, -autoVisionProcessing.sampleY, AngleUnit.DEGREES, -autoVisionProcessing.sampleAngles);

                        Mat processed = pipeline.processSingleFrame(autoVisionProcessing.frame);
                        Bitmap bitmap = Bitmap.createBitmap(processed.cols(), processed.rows(), Bitmap.Config.RGB_565);
                        Utils.matToBitmap(processed, bitmap);

                        dashboard.sendImage(bitmap);

                        AutoVisionProcessing.done = false;
                        autoVisionProcessing.currentState = AutoVisionProcessing.States.WAITING_TO_CAPTURE;
                        autoVisionProcessing.sampleX = 0.0;
                        autoVisionProcessing.sampleY = 0.0;
                        autoVisionProcessing.sampleAngles = 0.0;
                        Timer.reset();
                        visionState = VISIONSTATE.VISION_TURRET_GRAB;
                        break;
                    }
                }
                if (i == 2) {
                    Timer.reset();
                    visionState = VISIONSTATE.VISION_FINE_FAILED;
                    break;
                }
                break;

            case VISION_TURRET_GRAB:
                double a=-Math.asin(bestSample.relPos.x/RobotActionConfig.Turret_Arm_Length);
                Point fineError=new Point(
                        (pose2D.getX(DistanceUnit.INCH)*Math.cos(a))-(pose2D.getY(DistanceUnit.INCH)*Math.sin(a)),
                        (pose2D.getX(DistanceUnit.INCH)*Math.sin(a))+(pose2D.getY(DistanceUnit.INCH)*Math.cos(a))
                );

                advancedIntake.runToPoint(robot,
                        new Point(bestSample.relPos.x+fineError.x,bestSample.relPos.y+fineError.y)
                        , DistanceUnit.INCH);
                intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.OPEN;
                robot.intakeRotationServo.setPosition((RobotActionConfig.intake_Rotation_Mid*(1- (pose2D.getHeading(AngleUnit.DEGREES)/90))) + ((RobotActionConfig.intake_Turret_Mid + 0.2) - (robot.intakeTurretServo.getPosition()) + 0.2));

                if(Timer.seconds()>RobotActionConfig.intakeWristRotationTime) {
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                }
                //Change this delay to pickup time
                if(Timer.seconds()>RobotActionConfig.intakeWristRotationTime + RobotActionConfig.waitTime){
                    Timer.reset();
                    intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.CLOSE;
                    visionState = VISIONSTATE.IDLE;
                }

                break;

            case VISION_FINE_FAILED:
                robot.led.setPosition(0.3);

                robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);

                if(Timer.seconds()>RobotActionConfig.intakeWristRotationTime){
                    intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.CLOSE;
                    visionState = VISIONSTATE.IDLE;
                    break;
                }
            default:
                visionState = VISIONSTATE.IDLE;
        }
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}

class AutoVisionProcessing {
    private final FtcDashboard dashboard;
    private Servo led;
    private Pipeline pipeline;

    private final HardwareMap hardwareMap;

    public double sampleX = 0.0;
    public double sampleY = 0.0;
    public double sampleAngles = 0.0;

    public AutoVisionProcessing(FtcDashboard dashboard ,HardwareMap hardwareMap) {
        this.dashboard = dashboard;
        this.hardwareMap = hardwareMap;
    }

    public enum States {
        WAITING_TO_CAPTURE,
        CAPTURING,
        STATIC_PROCESSING,
        DONE
    }

    public States currentState = States.WAITING_TO_CAPTURE;
    int frameIndex;

    public static boolean done = false;
    public static boolean liveProcess = false;

    public Mat frame;

    public void initialize() {
        led = hardwareMap.get(Servo.class, "LED");
        led.setPosition(VisionConfigs.LED_BRIGHTNESS);

        pipeline = new Pipeline();
    }

    public void process(boolean liveProcessing) {
        led.setPosition(VisionConfigs.LED_BRIGHTNESS);

        switch (currentState) {
            case CAPTURING:
                done = false;

                if (liveProcessing) {
                    liveProcess = true;
                    if (pipeline.isDoneProcessing()) {
                        currentState = States.DONE;
                        break;
                    }
                }
                else if (pipeline.isDoneCapturing()) {
                    FiniteStateMachineVision.portal.stopStreaming();
                    currentState = States.STATIC_PROCESSING;
                    frameIndex = 0;
                }
                break;
            case STATIC_PROCESSING:
                List<Mat> frames = pipeline.frameBuffer;

                if (frameIndex < frames.size()) {
                    frame = frames.get(frameIndex);

                    frame.release();

                    frameIndex++;
                } else {
                    pipeline.clearBuffer();
                    currentState = States.DONE;
                }
                break;
            case DONE:
                done = true;
                sampleX = pipeline.realX;
                sampleY = pipeline.realY;
                sampleAngles = pipeline.angles;
                pipeline.releaseMemory();
                currentState = States.WAITING_TO_CAPTURE;
                break;
            default:
                currentState = States.WAITING_TO_CAPTURE;
                break;
        }
    }


}

class Pipeline implements VisionProcessor {
    public final List<Mat> frameBuffer = new ArrayList<>();
    public int MAX_FRAMES = 1;
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

    //Camera Lens Constants

    private final int cx = VisionConfigs.CAMERA_WIDTH / 2;
    private final int cy = VisionConfigs.CAMERA_HEIGHT / 2;

    private final double horizontalFOV = 70;
    private final double verticalFOV = (horizontalFOV * VisionConfigs.CAMERA_HEIGHT) / VisionConfigs.CAMERA_HEIGHT;

    //originally 228.5387
    private final double fx = ((double) VisionConfigs.CAMERA_WIDTH / 2) / Math.tan(Math.toRadians(horizontalFOV / 2)); //228.50368107873832034569896394533
    private final double fy = ((double) VisionConfigs.CAMERA_HEIGHT / 2) / Math.tan(Math.toRadians(verticalFOV / 2)); //243.33592823870697717256615973888

    //Sample Real Life Coordinates
    private final double REAL_WIDTH = 1.5;

    public double realX;
    public double realY;

    public double angles;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        if (AutoVisionProcessing.liveProcess) {
            return processSingleFrame(input);
        } else {
            if (!doneCapturing) {
                // Save a clone of the input frame
                frameBuffer.add(input.clone());
                if (frameBuffer.size() >= MAX_FRAMES) {
                    doneCapturing = true;
                }
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

    public Mat processSingleFrame (Mat inputFrame) {

        //Core.rotate(inputFrame, rotated, Core.ROTATE_180);

        angles = 0.0;
        realX = 0.0;
        realY = 0.0;
        doneProcessing = false;

        Imgproc.cvtColor(inputFrame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, VisionConfigs.RANGE_LOW, VisionConfigs.RANGE_HIGH, threshold);

        //Imgproc.GaussianBlur(threshold, blurred, new Size(3, 3), 0);

        Core.bitwise_and(inputFrame, inputFrame, masked, threshold);

        /** Can try true later **/
        Imgproc.Canny(masked, edged, Min_Val, Max_Val, 7, false);

        kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(3, 3));
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
        return inputFrame;
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

        if ((area < VisionConfigs.maxArea) && (area > VisionConfigs.minArea)) {
            maxHeap.add(rect);
            goodRect = maxHeap.poll();
        }

        return goodRect;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
    }
}

