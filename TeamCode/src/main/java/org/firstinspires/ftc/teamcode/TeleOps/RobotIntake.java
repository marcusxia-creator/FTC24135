package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class RobotIntake {
    //Declare IntakeState
    private enum IntakeState {
        INTAKE_EXTEND,
        INTAKE_GRAB,
        INTAKE_RETRACT,
        SAMPLE_TRANSFER
    }
    
    //Declare intake states
    private IntakeState intakeState = IntakeState.INTAKE_EXTEND;

    //Declare gamepad
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;

    private final RobotHardware robot;

    //Set up timer for debouncing
    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final ElapsedTime intakeTimer = new ElapsedTime();

    //Constructor
    public RobotIntake (RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void intakeSlideControl () {
        switch (intakeState) {
            case INTAKE_EXTEND:
                if (((gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) || 
                     (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) &&
                    debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extend);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Extend);
                    intakeState = IntakeState.INTAKE_GRAB;
                }
                else {
                    intakeInit();
                }
                break;
            case INTAKE_GRAB:
                if ((gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT)) && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Idle);
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_RETRACT;
                }
                else {
                    //Calling the methods
                    intakeArmControl();
                    intakeRotationServoSteer();
                }
                break;
            case INTAKE_RETRACT:
                if (intakeTimer.seconds() > RobotActionConfig.intake_Slide_Retract_Threshold) {
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                }
                if (intakeTimer.seconds() > RobotActionConfig.intake_Wrist_Arm_Retract_Threshold) {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    intakeState = IntakeState.SAMPLE_TRANSFER;
                }
                break;
            case SAMPLE_TRANSFER:
                if(intakeTimer.seconds() > RobotActionConfig.deposit_Claw_Close_Threshold) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                }
                if(intakeTimer.seconds() > RobotActionConfig.intake_Claw_Open_Threshold) {
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                }
                if(intakeTimer.seconds() > RobotActionConfig.intake_Arm_Idle_Threshold) {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    intakeState = IntakeState.INTAKE_EXTEND;
                }
                break;
            default:
                intakeState = IntakeState.INTAKE_EXTEND;
                break;
        }

        if ((gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ||
                (gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)) && debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
            robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
            robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
            intakeState = IntakeState.INTAKE_GRAB;
        }
    }

    public void intakeInit () {
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Idle);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
    }

    private void intakeArmControl () {
        //Rising edge detector
        if (gamepad_1.wasJustPressed(GamepadKeys.Button.DPAD_UP) || gamepad_2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            robot.intakeLeftArmServo.setPosition(robot.intakeLeftArmServo.getPosition() + RobotActionConfig.intake_Arm_Change_Amount);
            robot.intakeRightArmServo.setPosition(robot.intakeRightArmServo.getPosition() + RobotActionConfig.intake_Arm_Change_Amount);
        }
        if (gamepad_1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || gamepad_2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.intakeLeftArmServo.setPosition(robot.intakeLeftArmServo.getPosition() - RobotActionConfig.intake_Arm_Change_Amount);
            robot.intakeRightArmServo.setPosition(robot.intakeRightArmServo.getPosition() - RobotActionConfig.intake_Arm_Change_Amount);
        }
    }

    private void intakeRotationServoSteer() {
        //Rising edge detector
        if (gamepad_1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) || gamepad_2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() + RobotActionConfig.intake_Rotation_Steer_Amount);
        }
        if (gamepad_1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || gamepad_2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.intakeRotationServo.setPosition(robot.intakeRotationServo.getPosition() - RobotActionConfig.intake_Rotation_Steer_Amount);
        }
    }
}