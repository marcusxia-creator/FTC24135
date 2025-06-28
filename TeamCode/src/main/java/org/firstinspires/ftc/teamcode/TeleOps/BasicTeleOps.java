package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Auto.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Auto.drive.opmode.AutomaticFeedforwardTuner;
import org.firstinspires.ftc.teamcode.SingleFrameImageProcessing;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "TeleOps_Premier_2", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {

    public enum ControlState { RUN, TEST }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private FiniteStateMachineDeposit depositArmDrive;
    private FiniteStateMachineIntake intakeArmDrive;
    private ServoTest servoTest;
    private ControlState controlState = ControlState.RUN;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private boolean lBstartPressed = false;
    private List<LynxModule> allHubs;
    private SingleFrameImageProcessing pipeline;
    private OpenCvWebcam camera;
    private Servo led;
    //private Pose2D samplePose2D;

    private FtcDashboard dashboard;

    double sx;
    double sy;

    boolean imageStates = false;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        depositArmDrive = new FiniteStateMachineDeposit(robot, gamepadCo1, gamepadCo2, intakeArmDrive, telemetry);
        depositArmDrive.Init();


        intakeArmDrive = new FiniteStateMachineIntake(robot, gamepadCo1, gamepadCo2, depositArmDrive);
        intakeArmDrive.Init();

        servoTest = new ServoTest(robot, gamepadCo1, gamepadCo2);
        servoTest.init();

        pipeline = new SingleFrameImageProcessing();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        led = hardwareMap.get(Servo.class, "LED");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera =  OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDevice();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(pipeline.CAMERA_WIDTH, pipeline.CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN, OpenCvWebcam.StreamFormat.MJPEG);
                camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
                camera.getExposureControl().setExposure(7, TimeUnit.MILLISECONDS);
                camera.getGainControl().setGain(2);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        led.setPosition(1);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        telemetry.addLine("-------------------");
        telemetry.addData("Status", "initialized");
        telemetry.addData("Control Mode", robotDrive.getDriveMode().name());
        telemetry.addData("VS Left Encoder", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("VS Right Encoder", robot.liftMotorRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepadCo1.getButton(BACK) && debounceTimer.seconds() > 0.2) {
            debounceTimer.reset();
            resetLiftEncoders();
            depositArmDrive.Init();
        }

        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                if (hub.equals(allHubs.get(0))) {
                    telemetry.addData("Drive FL Pos", bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber()));
                    telemetry.addData("Drive FR Pos", bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber()));
                    telemetry.addData("Drive BL Pos", bulkData.getMotorCurrentPosition(robot.backLeftMotor.getPortNumber()));
                    telemetry.addData("Drive BR Pos", bulkData.getMotorCurrentPosition(robot.backRightMotor.getPortNumber()));
                } else {
                    telemetry.addData("Lift L Pos", bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber()));
                    telemetry.addData("Lift R Pos", bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber()));
                }
            }
        }

        robotDrive.DriveLoop();

        if (gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed) {
            toggleControlState();
            debounceTimer.reset();
            lBstartPressed = true;
        } else if (!gamepadCo1.getButton(START) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }

        if (controlState == ControlState.RUN) {
            depositArmDrive.DepositArmLoop();
            intakeArmDrive.IntakeArmLoop();

            telemetry.addData("Deposit State", depositArmDrive.liftState);
            telemetry.addData("Deposit Claw State", depositArmDrive.depositClawState);
            telemetry.addData("Intake State", intakeArmDrive.intakeState);
            telemetry.addData("Intake Claw State", intakeArmDrive.intakeClawState);
            /**telemetry.addData("Color Hue", RobotActionConfig.hsvValues[0]);
             telemetry.addData("Color Value", RobotActionConfig.hsvValues[2]); **/
        }

        if (controlState == ControlState.TEST) {
            servoTest.loop();
            if (!imageStates) {
                if (pipeline.isDoneCapturing()) {
                    camera.stopStreaming();

                    int frameIndex = 0;

                    List<Mat> frames = pipeline.frameBuffer;

                    if (frameIndex < frames.size()) {
                        Mat frame = frames.get(frameIndex);

                        Mat processed = pipeline.processSingleFrame(frame);
                        Bitmap bitmap = Bitmap.createBitmap(processed.cols(), processed.rows(), Bitmap.Config.RGB_565);
                        Utils.matToBitmap(processed, bitmap);

                        dashboard.sendImage(bitmap);

                        //sleep(1000);

                        processed.release();
                        frame.release();
                        bitmap.recycle();

                        frameIndex ++;
                    }
                    else {
                        pipeline.clearBuffer();
                    }
                    if (pipeline.realX != 0) {
                        imageStates = true;
                    }
                }
            }
            if (imageStates) {
                telemetry.addData("Frame Captured", pipeline.frameBuffer.size());
                telemetry.addLine("Camera Stopped");
                telemetry.addData("RealX", pipeline.realX);
            }
        }

        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Drive Mode", robotDrive.getDriveMode().name());
        telemetry.addData("VS Left", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("VS Right", robot.liftMotorRight.getCurrentPosition());
        telemetry.addData("Deposit Arm", robot.depositLeftArmServo.getPosition());
        telemetry.addData("Deposit Wrist", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw", robot.depositClawServo.getPosition());
        telemetry.addData("Intake Arm", robot.intakeArmServo.getPosition());
        telemetry.addData("Intake Wrist", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Claw", robot.intakeClawServo.getPosition());
        telemetry.addData("Intake Slide L", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide R", robot.intakeRightSlideServo.getPosition());
        telemetry.addData("Heading", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addLine("--------------Servo Test--------------");
        // --- Telemetry ---
        telemetry.addData("VS Left Position", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("VS Right Position", robot.liftMotorRight.getCurrentPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("Deposit Arm Position", robot.depositLeftArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw Position", robot.depositClawServo.getPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("Intake Arm Left Position", robot.intakeArmServo.getPosition());
        telemetry.addData("Intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addData("Intake Slide Left Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide Right Position", robot.intakeRightSlideServo.getPosition());
        telemetry.addData("Intake Turret Position", robot.intakeTurretServo.getPosition());
        telemetry.addData("Intake Rotation Position", robot.intakeRotationServo.getPosition());
        telemetry.addData("Sample realX", sx);
        telemetry.addData("Sample realY", sy);
        //telemetry.addData("Sample Angle", samplePose2D.heading);

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }

    private void toggleControlState() {
        controlState = (controlState == ControlState.RUN) ? ControlState.TEST : ControlState.RUN;
    }

    /**
    private boolean LSisPressed() {
        return robot.limitSwitch.getState();
    }
     */

    private void resetLiftEncoders() {
        robot.liftMotorLeft.setTargetPosition(0);
        robot.liftMotorRight.setTargetPosition(0);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.3);
        robot.liftMotorRight.setPower(0.3);

        /**
        while (robot.liftMotorLeft.isBusy() && robot.liftMotorRight.isBusy()) {
            if (LSisPressed()) {
                robot.liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                break;
            }
        }
         */
    }
}
