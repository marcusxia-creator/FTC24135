package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class FiniteMachineStateIntake {
    private final GamepadEx gamepad;
    private final RobotHardware robot;
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses

    final double intake_Arm_Idle;     // intake when retract
    final double intake_Arm_Pick;  // intake arm for pick
    final double intake_Arm_Trans;
    final double intake_Slide_Retract; // intake slide retract for drop to transfer
    final double intake_Slide_Extend;  // intake slide extend for pick
    final double intake_Rotation_Mid;// rotation position
    final double intake_Claw_Open;
    final double intake_Claw_Close;

    private double intakeArmPosition;
    private double rotationPosition;

    public FiniteMachineStateIntake(RobotHardware robot, GamepadEx gamepad,
                                    double intake_Arm_Idle, double intake_Arm_Pick,double intake_Arm_Trans,
                                    double intake_Slide_Retract, double intake_Slide_Extend,
                                    double intake_Rotation_Mid,
                                    double intake_Claw_Open, double intake_Claw_Close) {
        this.gamepad = gamepad;
        this.robot = robot;

        this.intake_Arm_Idle = intake_Arm_Idle;
        this.intake_Arm_Pick = intake_Arm_Pick;
        this.intake_Arm_Trans = intake_Arm_Trans;
        this.intake_Slide_Retract = intake_Slide_Retract;
        this.intake_Slide_Extend = intake_Slide_Extend;
        this.intake_Rotation_Mid = intake_Rotation_Mid;
        this.intake_Claw_Open = intake_Claw_Open;
        this.intake_Claw_Close = intake_Claw_Close;
    }

    public enum INTAKESTATE {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_RETRACT,
        INTAKE_TRANS
    }

    public INTAKESTATE intakestate = INTAKESTATE.INTAKE_START; // Persisting state
    private ElapsedTime intakeTimer = new ElapsedTime(); // Timer for controlling dumping time
    private CLAWSTATE clawstate = CLAWSTATE.OPEN; //claw default open


    public void Init() {
        intakeTimer.reset();
        robot.intakeSlideServo.setPosition(intake_Slide_Retract);
        robot.intakeRightArmServo.setPosition(intake_Arm_Trans);
        robot.intakeLeftArmServo.setPosition(intake_Arm_Trans);
        robot.intakeRotationServo.setPosition(intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(intake_Claw_Open);
    }

    public void IntakeArmLoop() {
        // Display current lift state and telemetry feedback
        switch (intakestate) {
            case INTAKE_START:
                // Debounce the button press for starting the lift extend
                robot.intakeClawServo.setPosition(intake_Claw_Open);
                if (gamepad && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                    debounceTimer.reset();
                    robot.intakeSlideServo.setPosition(intake_Slide_Extend);
                    robot.intakeRotationServo.setPosition(intake_Rotation_Mid);
                    robot.intakeLeftArmServo.setPosition(intake_Arm_Pick);
                    robot.intakeRightArmServo.setPosition(intake_Arm_Pick);
                    robot.intakeClawServo.setPosition(intake_Claw_Open);
                    intakeTimer.reset();
                    intakestate = INTAKESTATE.INTAKE_EXTEND;
                }
                break;
            case INTAKE_EXTEND:
                // Check if the lift has reached the high position
                rotationPosition = robot.intakeRotationServo.getPosition();
                intakeArmPosition = (robot.intakeRightArmServo.getPosition()+robot.intakeLeftArmServo.getPosition())/2;

                if (intakeTimer.seconds()> 0.5) {

                    // claw rotation
                    if (gamepad.getButton(LEFT_BUMPER) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                        debounceTimer.reset();
                        //use to be 0.01
                        rotationPosition += 0.05;
                        robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                        robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                    }
                    //claw rotation
                    if (gamepad.getButton(RIGHT_BUMPER) && (debounceTimer.seconds() > DEBOUNCE_THRESHOLD)) {
                        debounceTimer.reset();
                        //use to be 0.01
                        rotationPosition -= 0.05;
                        robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                        robot.intakeRotationServo.setPosition(Range.clip(rotationPosition, 0, 1));
                    }

                    // add in the button for claw open and close
                    if(gamepad.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                        debounceTimer.reset();
                        ToggleClaw();
                    }
                    if (clawstate == CLAWSTATE.OPEN){
                        robot.intakeClawServo.setPosition(intake_Claw_Open);
                    } else{
                        robot.intakeClawServo.setPosition(intake_Claw_Close);
                    }

                    // intake retract.
                    if (gamepad.getButton(DPAD_LEFT) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
                        debounceTimer.reset();
                        //retract
                        intakeTimer.reset();
                        intakestate = INTAKESTATE.INTAKE_RETRACT;
                    }
                }
                break;

            case INTAKE_RETRACT:
                // Wait for the dump time to pass
                robot.intakeRotationServo.setPosition(intake_Rotation_Mid);
                robot.intakeLeftArmServo.setPosition(intake_Arm_Idle);
                robot.intakeRightArmServo.setPosition(intake_Arm_Idle);
                if (intakeTimer.seconds()>0.5){
                    robot.intakeSlideServo.setPosition(intake_Slide_Retract);
                    intakeTimer.reset();
                    intakestate = INTAKESTATE.INTAKE_TRANS;
                }
                break;

            case INTAKE_TRANS:
                // Check if the lift has reached the low position
                if(intakeTimer.seconds()>= 0.5) {
                    robot.intakeLeftArmServo.setPosition(intake_Arm_Trans);
                    robot.intakeRightArmServo.setPosition(intake_Arm_Trans);
                }
                if(intakeTimer.seconds()>= 1) {
                    robot.intakeClawServo.setPosition(intake_Claw_Open);
                }
                if(intakeTimer.seconds()>= 1.1) {
                    robot.depositClawServo.setPosition(BasicTeleOps.deposit_Claw_Close);
                }
                if(intakeTimer.seconds()>= 1.3) {
                    robot.intakeLeftArmServo.setPosition(0.32);
                    robot.intakeRightArmServo.setPosition(0.32);
                    intakestate = INTAKESTATE.INTAKE_START;
                }
                break;
            default:
                intakestate = INTAKESTATE.INTAKE_START;
                break;
        }

        // Handle lift cancel action if 'Y' button is pressed
        if (gamepad.getButton(GamepadKeys.Button.B) && intakestate != INTAKESTATE.INTAKE_START) {
            intakestate = INTAKESTATE.INTAKE_START;
        }

        //intake arm up
        if (gamepad.getButton(DPAD_UP) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            //use to be 0.01
            intakeArmPosition += 0.05;
            robot.intakeLeftArmServo.setPosition(Range.clip(intakeArmPosition, 0, 0.55));
            robot.intakeRightArmServo.setPosition(Range.clip(intakeArmPosition, 0, 0.55));
        }

        //intake arm down
        if (gamepad.getButton(DPAD_DOWN) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            //use to be 0.01
            intakeArmPosition -= 0.05;
            robot.intakeLeftArmServo.setPosition(Range.clip(intakeArmPosition, 0.0, 0.55));
            robot.intakeRightArmServo.setPosition(Range.clip(intakeArmPosition, 0.0, 0.55));
        }
    }

    // Helper method to check if the lift is within the desired position threshold
    private boolean isSlideAtPosition(double targetPosition) {
        return Math.abs(robot.intakeSlideServo.getPosition() - targetPosition) < 0.05;
    }

    // for return intakestate for telemetry
    INTAKESTATE intakeState(){
        return intakestate;
    }
    public enum CLAWSTATE {
        OPEN,
        CLOSE
    }
    private void ToggleClaw() {
        if (clawstate == CLAWSTATE.OPEN) {
            clawstate = CLAWSTATE.CLOSE;
        } else {
            clawstate = CLAWSTATE.OPEN;
        }
    }
}
