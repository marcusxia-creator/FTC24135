package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Button Config for intake
 *Start state
 * *DPad Right          : retract/extend

 *Pick state
 * *Dpad up             : rise intake arm
 * *Dpad down           : lower intake arm
 * *left bumper         : to rotate left
 * *right bumper        : to rotate right

 * universal state
 * *Right Trigger + DPADRIGHT : Lower the intake arm only
 * *A                   : to open/close intake

 * Action for intake
 * *default open intake when extend
 * *default close intake when retract
 */
public class FiniteStateMachineIntake {

    //Intake STATE
    public enum INTAKESTATE {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_PICK,
        INTAKE_RETRACT,
        INTAKE_TRANS
    }

    // Robot and Gamepad Member
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private final FiniteStateMachineDeposit depositArmDrive;

    //Time member
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    //Intake states
    public INTAKESTATE intakeState = INTAKESTATE.INTAKE_START; // Persisting state
    private ElapsedTime intakeTimer = new ElapsedTime(); // Timer for controlling dumping time
    public INTAKECLAWSTATE inTakeclawState ; //claw default open

    private double intakeArmPosition;
    private double rotationPosition;
    FiniteStateMachineDeposit.LIFTSTATE depositArmState;



    public FiniteStateMachineIntake(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineDeposit depositArmDrive) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.depositArmDrive = depositArmDrive;
        this.inTakeclawState = INTAKECLAWSTATE.OPEN;
    }

    //Initialization
    public void Init() {
        intakeTimer.reset();
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Initial);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
    }

    //FSM Loop Control
    public void IntakeArmLoop() {
        // Display current lift state and telemetry feedback
        switch (intakeState) {
            case INTAKE_START:
                /** Debounce the button press 'DPAD_RIGHT' for starting the lift extend */
                if (((gamepad_1.getButton(DPAD_RIGHT) && gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1)||
                        (gamepad_2.getButton(DPAD_RIGHT)&& gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1))&&
                        debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Extension);
                    robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                    //robot.intakeClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    inTakeclawState = INTAKECLAWSTATE.OPEN;
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_EXTEND;
                }
                break;
            case INTAKE_EXTEND:
                // after 0.5s intake arm lower for pick up
                if (intakeTimer.seconds()> 0.5) {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_PICK;
                }
            case INTAKE_PICK:
                rotationPosition = robot.intakeRotationServo.getPosition();
                if (intakeTimer.seconds()> 0.25) {
                    /** claw rotation - LEFT BUMPER */
                    if ((gamepad_1.getButton(LEFT_BUMPER) || gamepad_2.getButton(LEFT_BUMPER)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                        debounceTimer.reset();
                        //use to be 0.01
                        rotationPosition += 0.15;
                        robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                    }
                    /** claw rotation RIGHT BUMPER */
                    if ((gamepad_1.getButton(RIGHT_BUMPER) || gamepad_2.getButton(RIGHT_BUMPER))&& (debounceTimer.seconds() > DEBOUNCE_THRESHOLD)) {
                        debounceTimer.reset();
                        //use to be 0.01
                        rotationPosition -= 0.15;
                        robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                    }

                    /** intake arm up DPAD UP */
                    if ((gamepad_1.getButton(DPAD_UP) || gamepad_2.getButton(DPAD_UP)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                        debounceTimer.reset();
                        //use to be 0.01
                        intakeArmPosition = (robot.intakeRightArmServo.getPosition());
                        intakeArmPosition -= 0.05;
                        robot.intakeLeftArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                        robot.intakeRightArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                    }

                    /** intake arm down DPAD DOWN */
                    if ((gamepad_1.getButton(DPAD_DOWN) || gamepad_2.getButton(DPAD_DOWN)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                        debounceTimer.reset();
                        //use to be 0.01
                        intakeArmPosition = (robot.intakeRightArmServo.getPosition());
                        intakeArmPosition += 0.05;
                        robot.intakeLeftArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                        robot.intakeRightArmServo.setPosition(Range.clip(intakeArmPosition, 0.1, 0.42));
                    }

                    /** intake retract
                     * DPAD_RIGHT again
                     */
                    if ((gamepad_1.getButton(DPAD_RIGHT) || gamepad_2.getButton(DPAD_RIGHT))&& debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                        debounceTimer.reset();
                        inTakeclawState = INTAKECLAWSTATE.CLOSE;
                        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
                        //retract
                        intakeTimer.reset();
                        intakeState = INTAKESTATE.INTAKE_RETRACT;
                    }
                }
                break;

            case INTAKE_RETRACT:
                // Wait for the pickup time to pass
                if (intakeTimer.seconds()>0.2){
                robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
                robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Idle);
                robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
                }
                if (intakeTimer.seconds()>0.4){
                    robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
                    intakeTimer.reset();
                    intakeState = INTAKESTATE.INTAKE_TRANS;
                }
                break;

            case INTAKE_TRANS:
                // read in deposit arm state
                depositArmState = depositArmDrive.State();
                // Check if the lift has reached the low position
                if(intakeTimer.seconds()>0.2) {
                    robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer); // intake wrist reach transfer pos first.
                }
                //0.2 sec after wrist at transfer pos, intake arm move to transfer pos.
                if(intakeTimer.seconds()>= 0.4) {
                    robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                    robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
                }
                if(depositArmState == FiniteStateMachineDeposit.LIFTSTATE.LIFT_START && intakeTimer.seconds()>= 0.8) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
                    depositArmDrive.SetDepositClawState(FiniteStateMachineDeposit.DEPOSITCLAWSTATE.CLOSE);
                }
                if (intakeTimer.seconds() >= 0.9){
                    robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
                }
                break;

            default:
                intakeState = INTAKESTATE.INTAKE_START;
                break;
        }

        /** lower intake arm only for grabbing - right tigger + DPAD_RIGHT */
        if ((gamepad_1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ||
                (gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.6 && gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)) &&
                        debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            inTakeclawState = INTAKECLAWSTATE.OPEN;
            IntakeClawSwitch();
            intakeState = INTAKESTATE.INTAKE_EXTEND;
        }

        /** Claw control - Button A */
        // add in the button "A" for intake claw open and close
        if((gamepad_1.getButton(GamepadKeys.Button.A) || gamepad_2.getButton(GamepadKeys.Button.A))&& debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            ToggleClaw();
            IntakeClawSwitch();
        }

    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean IsSlideAtPosition(double targetPosition) {
        return Math.abs(robot.intakeLeftSlideServo.getPosition() - targetPosition) < 0.005;
    }

    // for return intakeState for telemetry
    public INTAKESTATE intakeState(){
        return intakeState;
    }

    /** Claw control
     * Claw state
     * Claw Toggle
     * Claw open / close
     * */
    //Claw State
    public enum INTAKECLAWSTATE {
        OPEN,
        CLOSE
    }
    // set claw state
    public void SetInTakeClawState(INTAKECLAWSTATE state) {
        this.inTakeclawState = state;
    }

    //Toggle Claw()
    private void ToggleClaw() {
        if (inTakeclawState == INTAKECLAWSTATE.OPEN) {
            inTakeclawState = INTAKECLAWSTATE.CLOSE;
        } else {
            inTakeclawState = INTAKECLAWSTATE.OPEN;
        }
    }

    private void IntakeClawSwitch(){
        if (inTakeclawState == INTAKECLAWSTATE.OPEN){
            robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        } else{
            robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);
        }
    }
}
