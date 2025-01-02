package org.firstinspires.ftc.teamcode.TeleOps;

import static java.lang.Thread.sleep;

import android.graphics.Color;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Button Config for deposit
 * *X                   : extend
 * *B                   : Cancel
 */

public class FiniteMachineStateArm {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    
    public enum LIFTSTATE {
        LIFT_START,
        LIFT_TRANSFER,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT,
        LIFT_HIGHTBAR,
        LIFT_HOOK,
    }

    private DEPOSITSTATE depositState;
    
    private LIFTSTATE liftState = LIFTSTATE.LIFT_START; // Persisting state
    private ElapsedTime liftTimer = new ElapsedTime(); // Timer for controlling dumping time
 
    private ElapsedTime transfer_timer = new ElapsedTime();
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    private boolean black_color;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    public FiniteMachineStateArm(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }
    // Initialize Deposit Arm
    public void Init() {
        liftTimer.reset();
        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.1);                                          // Make sure lift motor is on
        robot.liftMotorRight.setPower(0.1);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Retract_Pos);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Retract_Pos);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
    }

    // Deposit Arm Control
    public void DepositArmLoop() throws InterruptedException {
        // Display current lift state and telemetry feedback
        switch (liftState) {
            case LIFT_START:
                // Debounce the button press X for starting the lift extend
                black_color = false;
                Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);
                if (((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)||
                        (gamepad_2.getButton(GamepadKeys.Button.X)&& gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)) && !black_color &&
                debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    transfer_timer.reset();
                    liftState = LIFTSTATE.LIFT_TRANSFER;
                }

                // "right trigger" and "X" button and color black to set deposit arm to pick specimen position
                if (((gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6)||
                        (gamepad_2.getButton(GamepadKeys.Button.X)&& gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6)) &&
                        debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    transfer_timer.reset();
                    while (!black_color){
                        if (hsvValues[0] < 77 && hsvValues[0] > 73){
                            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                            depositState = DEPOSITSTATE.CLOSE;
                        }
                        ClawControl();
                        if(depositState == DEPOSITSTATE.CLOSE){
                            black_color =!black_color;
                        }
                    }
                    liftState = LIFTSTATE.LIFT_HIGHTBAR;
                }
                break;

            case LIFT_TRANSFER:
                if (((hsvValues[0] < 77 && hsvValues[0] > 73)
                        || (hsvValues[0] < 20 && hsvValues[0]>16)
                        || (hsvValues[0]<228 && hsvValues[0]>224) && hsvValues[1] > 0.5)) {

                    if (transfer_timer.milliseconds() > 200) {
                        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                    }

                    if (transfer_timer.milliseconds() > 300) {
                        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    }

                    if (transfer_timer.milliseconds() > 400) {
                        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
                        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
                    }

                    if (transfer_timer.milliseconds() >= 850) {
                        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                        robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                        liftState = LIFTSTATE.LIFT_EXTEND;
                    }
                }
                else{
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;
            case LIFT_EXTEND:
                // Check if the lift has reached the high position
                if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos)) {
                    //move deposit arm to dump
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump_Pos);
                   // Move deposit wrist servo to dump position
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump_Pos);
                    liftTimer.reset();
                    liftState = LIFTSTATE.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                // Wait for the dump time to pass
                if (liftTimer.seconds() >= RobotActionConfig.dumpTime) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    depositState = DEPOSITSTATE.OPEN;
                }
                if (liftTimer.seconds() >= RobotActionConfig.dumpTime+0.5) {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Retract_Pos);// Reset servo to idle
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Retract_Pos);
                    liftState = LIFTSTATE.LIFT_RETRACT;
                }
                break;

            case LIFT_RETRACT:
                // Check if the lift has reached the low position
                if(Servo_AtPosition(RobotActionConfig.deposit_Claw_Open) && liftTimer.seconds()>= RobotActionConfig.retractTime) {
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos); // Start retracting the lift
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos); // Start retracting the lift
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                }
                if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Down_Pos)) {
                    robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                    robot.liftMotorRight.setPower(0);
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;

            case LIFT_HIGHTBAR:
                if(gamepad_1.getButton(GamepadKeys.Button.X) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1 && black_color){
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Pos); // Start Rise to highbar position
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Pos); // Start retracting the lift
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    transfer_timer.reset();
                    liftState = LIFTSTATE.LIFT_HOOK;
                    liftTimer.reset();
                }
                break;
            case LIFT_HOOK:
                if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Highbar_Pos)){
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos); // Start retracting the lift
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Down_Pos); // Start retracting the lift
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    if(liftTimer.seconds()>= 0.15) {
                        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                        depositState = DEPOSITSTATE.OPEN;
                    }
                }
                if (IsLiftAtPosition(RobotActionConfig.deposit_Slide_Down_Pos)) {
                    robot.liftMotorLeft.setPower(0); // Stop the motor after reaching the low position
                    robot.liftMotorRight.setPower(0);
                    liftState = LIFTSTATE.LIFT_START;
                }
                break;
            default:
                liftState = LIFTSTATE.LIFT_START;
                break;
        }

        // Handle lift Cancel Action if 'B' button is pressed
        if ((gamepad_1.getButton(GamepadKeys.Button.B) || gamepad_2.getButton(GamepadKeys.Button.B)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD && liftState != LIFTSTATE.LIFT_START) {
            debounceTimer.reset();
            liftState = LIFTSTATE.LIFT_START;
            robot.liftMotorLeft.setPower(0); // Ensure the motor is stopped
            robot.liftMotorRight.setPower(0);
            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Retract_Pos);
            robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Retract_Pos);
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        }

        // Claw control - Button Back
        ClawControl();
    }

    private void ClawControl(){
        if((gamepad_1.getButton(GamepadKeys.Button.Y) || gamepad_2.getButton(GamepadKeys.Button.Y))&& debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
            debounceTimer.reset();
            ToggleDeposit();
            if (depositState == DEPOSITSTATE.OPEN) {
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
            } else {
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
            }
        }
    }
    // Helper method to check if the lift is within the desired position threshold
    private boolean IsLiftAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 5 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 5;
    }

    private boolean Servo_AtPosition(double servoClawPosition) {
        return Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) < 0.01;
    }
    LIFTSTATE State(){
        return liftState;
    }

    //Deposit Claw State
    public enum DEPOSITSTATE {
        OPEN,
        CLOSE
    }

    //Toggle Deposit Claw Open - Close
    private void ToggleDeposit() {
        if (depositState == DEPOSITSTATE.OPEN) {
            depositState = DEPOSITSTATE.CLOSE;
        } else {
            depositState = DEPOSITSTATE.OPEN;
        }
    }
}
