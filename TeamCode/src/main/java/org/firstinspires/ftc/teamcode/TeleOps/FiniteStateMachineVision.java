package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

public class FiniteStateMachineVision {

    public enum VISIONSTATE{
        IDLE,

        VISION_COARSE_DETECT,
        VISION_COARSE_EXTEND,

        VISION_FINE
    }

    public VISIONSTATE visionState;

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;
    private final FiniteStateMachineIntake intakeArmDrive;

    public FiniteStateMachineVision(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2, FiniteStateMachineIntake intakeArmDrive) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
        this.intakeArmDrive = intakeArmDrive;
        this.visionState = VISIONSTATE.IDLE;
    }

    public void Init(boolean detectBlue,boolean detectRed, boolean detectYellow){

    }

    public void VisionLoop(){
        switch(visionState){
            case IDLE:

                break;

            case VISION_COARSE_DETECT:

                break;

            case VISION_COARSE_EXTEND:

                break;

            case VISION_FINE:

                break;
        }
    }
}
