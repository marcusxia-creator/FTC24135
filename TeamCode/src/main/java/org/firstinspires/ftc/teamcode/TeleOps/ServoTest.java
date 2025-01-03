package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ServoTest {

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;

    // Declare deposit servo positions
    public static double depositLeftArm = 0.5;
    public static double depositRightArm = 0.5;
    private double servoposition;
    private int iniPosition = 50;
    private int deltaPosition = 50;
    private int currentPosition;
    private static final double speed = 0.4;

    private final ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    public ServoTest(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void ServoTestInit() {
        robot.depositArmServo.setPosition(0.0);
        robot.depositWristServo.setPosition(0.0);
        robot.depositClawServo.setPosition(0.0);

        robot.intakeLeftArmServo.setPosition(0.0);
        robot.intakeRightArmServo.setPosition(0.0);
        robot.intakeSlideLeftServo.setPosition(0.0);
        robot.intakeSlideRightServo.setPosition(0.0);

        depositLeftArm = robot.depositArmServo.getPosition();
        depositRightArm = robot.depositArmServo.getPosition();

        robot.liftMotorLeft.setTargetPosition(iniPosition);
        robot.liftMotorRight.setTargetPosition(iniPosition);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
    }

    public void ServoTestLoop() {
        // Gamepad 1 - Deposit Claw Servo
        if (gamepad_1.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositClawServo.getPosition() + 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(GamepadKeys.Button.B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositClawServo.getPosition() - 0.01;
            robot.depositClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // Gamepad 1 - Deposit Arm Servo
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositArmServo.getPosition() + 0.01;
            robot.depositArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositArmServo.getPosition() - 0.01;
            robot.depositArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // Gamepad 1 - Deposit Wrist Servo
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition() + 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.depositWristServo.getPosition() - 0.01;
            robot.depositWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // Gamepad 1 - Lift Motor Adjustment
        if (gamepad_1.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            currentPosition = robot.liftMotorLeft.getCurrentPosition();
            robot.liftMotorLeft.setTargetPosition(Range.clip(currentPosition + deltaPosition, 50, 3000));
            robot.liftMotorRight.setTargetPosition(Range.clip(currentPosition + deltaPosition, 50, 3000));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            currentPosition = robot.liftMotorLeft.getCurrentPosition();
            robot.liftMotorLeft.setTargetPosition(Range.clip(currentPosition - deltaPosition, 50, 3000));
            robot.liftMotorRight.setTargetPosition(Range.clip(currentPosition - deltaPosition, 50, 3000));
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(speed);
            robot.liftMotorRight.setPower(speed);
        }

        // Gamepad 2 - Intake Servos
        if (gamepad_2.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeClawServo.getPosition() + 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(GamepadKeys.Button.B) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeClawServo.getPosition() - 0.01;
            robot.intakeClawServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // Gamepad 2 - Intake Left/Right Arm Servos
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeLeftArmServo.getPosition() + 0.01;
            robot.intakeLeftArmServo.setPosition(Range.clip(servoposition, 0, 1));
            servoposition = robot.intakeRightArmServo.getPosition() + 0.01;
            robot.intakeRightArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeLeftArmServo.getPosition() - 0.01;
            robot.intakeLeftArmServo.setPosition(Range.clip(servoposition, 0, 1));
            servoposition = robot.intakeRightArmServo.getPosition() - 0.01;
            robot.intakeRightArmServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        // Gamepad 2 - Wrist and Rotation Servos
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition() - 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeWristServo.getPosition() + 0.01;
            robot.intakeWristServo.setPosition(Range.clip(servoposition, 0, 1));
        }

        if (gamepad_2.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition() + 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.Y) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            servoposition = robot.intakeRotationServo.getPosition() - 0.01;
            robot.intakeRotationServo.setPosition(Range.clip(servoposition, 0, 1));
        }
    }
}
