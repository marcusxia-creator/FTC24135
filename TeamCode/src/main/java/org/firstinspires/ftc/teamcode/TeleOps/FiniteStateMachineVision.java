package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import android.util.Size;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.FindBestSample;
import org.firstinspires.ftc.teamcode.TeleOps.coarsevisionproc.Sample;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Point;

import java.util.ArrayList;

public class FiniteStateMachineVision {

    public enum VISIONSTATE {
        IDLE,

        VISION_COARSE_DETECT,
        VISION_COARSE_EXTEND,

        VISION_FINE_LIVE,
        VISION_FINE_STATIC,

        VISION_TURRET_GRAB,
        VISION_FINE_FAILED,

        ROBOT_RESET
    }

    public VISIONSTATE visionState;

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private final FiniteStateMachineIntake intakeArmDrive;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    private AutoVisionProcessing autoVisionProcessing;
    private FtcDashboard dashboard;
    private HardwareMap hardwareMap;

    private Pose2D pose2D;

    private ArrayList<ColorBlobLocatorProcessor> useProcessors;
    private VisionPortal portal;

    private ElapsedTime Timer = new ElapsedTime();

    public Sample bestSample;

    public boolean takeControls;

    private int i = 0;

    public FiniteStateMachineVision(FtcDashboard dashboard,RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineIntake intakeArmDrive,boolean takeControls) {
        this.dashboard = dashboard;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.intakeArmDrive = intakeArmDrive;
        this.visionState = VISIONSTATE.IDLE;
        this.takeControls=  takeControls ;
    }

    public void init(boolean detectBlue, boolean detectRed, boolean detectYellow){
        autoVisionProcessing = new AutoVisionProcessing(dashboard, hardwareMap);
        autoVisionProcessing.initialize();

        ColorBlobLocatorProcessor blueColorLocator= FindBestSample.initProcessor(org.firstinspires.ftc.vision.opencv.ColorRange.BLUE);
        ColorBlobLocatorProcessor yellowColorLocator=FindBestSample.initProcessor(org.firstinspires.ftc.vision.opencv.ColorRange.YELLOW);
        ColorBlobLocatorProcessor redColorLocator=FindBestSample.initProcessor(ColorRange.RED);

        useProcessors=new ArrayList<>();

        portal = new VisionPortal.Builder()
                .addProcessors(blueColorLocator,yellowColorLocator,redColorLocator)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(robot.Webcam)
                .build();

        if(detectBlue)  {useProcessors.add(blueColorLocator);}
        if(detectRed)   {useProcessors.add(redColorLocator);}
        if(detectYellow){useProcessors.add(yellowColorLocator);}

    }

    public void visionLoop(boolean isFirstTimeDetecting) {
        switch(visionState){
            case IDLE:
                if(!takeControls){
                    visionState=VISIONSTATE.VISION_COARSE_DETECT;
                }
                //Change key
                if (((((gamepad_1.getButton(DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(DPAD_RIGHT) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1))

                        || (gamepad_1.getButton(DPAD_LEFT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) ||
                        (gamepad_2.getButton(DPAD_LEFT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                        isButtonDebounced())) {
                    visionState=VISIONSTATE.VISION_COARSE_DETECT;
                }
                break;

            case VISION_COARSE_DETECT:

                robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Coarse);
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Coarse);
                advancedIntake.runToPoint(robot,new Point(0,0), DistanceUnit.INCH);

                intakeArmDrive.intakeState = FiniteStateMachineIntake.INTAKESTATE.INTAKE_DISABLED;

                bestSample=FindBestSample.findBestSample(useProcessors,VisionConfigs.CamPos,VisionConfigs.Arducam);

                Timer.reset();

                if(bestSample!=null) {
                    if (bestSample.relPos.x > -7 &
                            bestSample.relPos.x < 7 &
                            bestSample.relPos.y > 0 &
                            bestSample.relPos.y < 14
                    ) {
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
                            bestSample.relPos.y < 14
                    ) {
                        intakeArmDrive.intakeState = FiniteStateMachineIntake.INTAKESTATE.INTAKE_DISABLED;
                        advancedIntake.runToPoint(robot, bestSample.relPos, DistanceUnit.INCH);
                        robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Fine);
                        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Fine);
                    }
                }

                if(Timer.seconds()>RobotActionConfig.intakeSlideExtendTime){
                    visionState = VISIONSTATE.VISION_FINE_STATIC;
                }

                break;

            case VISION_FINE_LIVE:

                break;

            case VISION_FINE_STATIC:
                robot.led.setPosition(VisionConfigs.LED_BRIGHTNESS);
                intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.OPEN;
                autoVisionProcessing.currentState = AutoVisionProcessing.States.CAPTURING;

                robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Fine);
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Fine);

                for (i = 0; i < VisionConfigs.MAX_FRAMES; i++) {

                    autoVisionProcessing.process();

                    if (autoVisionProcessing.done && autoVisionProcessing.sampleAngles != 0.0) {
                        pose2D = new Pose2D(DistanceUnit.INCH, autoVisionProcessing.sampleX, autoVisionProcessing.sampleY, AngleUnit.DEGREES, autoVisionProcessing.sampleAngles);
                        autoVisionProcessing.done = false;
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
                    visionState = VISIONSTATE.VISION_FINE_FAILED;
                    Timer.reset();
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
                robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid*(1- (pose2D.getHeading(AngleUnit.DEGREES)/90)));

                if(Timer.seconds()>RobotActionConfig.intakeWristRotationTime){
                    Timer.reset();
                    visionState=VISIONSTATE.IDLE;
                    robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    //add delay somehow
                    intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.CLOSE;
                }

                break;

            case VISION_FINE_FAILED:
                robot.intakeArmServo.setPosition(RobotActionConfig.intake_Arm_Grab);
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Grab);

                if(Timer.seconds()>RobotActionConfig.intakeWristRotationTime){
                    intakeArmDrive.intakeClawState= FiniteStateMachineIntake.INTAKECLAWSTATE.CLOSE;
                }

            case ROBOT_RESET:
                robot.led.setPosition(0.3);
                intakeArmDrive.Init();
                visionState = VISIONSTATE.IDLE;
                break;

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
