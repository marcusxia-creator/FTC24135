package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/* Button Config for deposit
 * *X                   : extend
 * *B                   : Cancel
 */


public class FiniteStateMachineArm {
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private RobotHardware robot;



    public enum DEPOSITCONTROLSTATE {
        START,
        BASKET_SLIDE_EXTEND,
        BASKET_ARM_EXTEND,
        BASKET_DUMP,
        BASKET_RETRACT,
        PICK_UP_ARM_EXTEND,
        PICK_UP_DROP,
        BAR_SLIDE_EXTEND,
        BAR_ARM_EXTEND,
        BAR_SCORE,
        BAR_RETRACT
    }


    private DEPOSITCLAWSTATE depositState;

    private DEPOSITCONTROLSTATE depositControlState = DEPOSITCONTROLSTATE.START; // Persisting state
    private ElapsedTime liftTimer = new ElapsedTime();// Timer for controlling dumping time

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private final double DEBOUNCE_THRESHOLD = 0.2; // Debouncing threshold for button presses


    public FiniteStateMachineArm(RobotHardware robot, GamepadEx gamepad_1,
                                 GamepadEx gamepad_2) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;
    }

    // claw close

    // Initialize Deposit Arm
    public void Init() {
        liftTimer.reset();
        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(0.1);                                          // Make sure lift motor is on
        robot.liftMotorRight.setPower(0.1);
        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer_Pos);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer_Pos);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
    }

    // Deposit Arm Control
    public void DepositControl() {
        // Display current lift state and telemetry feedback
        switch (depositControlState) {
            // start position
            case START:
                debounceTimer.reset();
                liftTimer.reset();
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer_Pos);
                robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer_Pos);
                if (gamepad_1.getButton(GamepadKeys.Button.X)&&debounceTimer.seconds()>DEBOUNCE_THRESHOLD) {
                    depositControlState = DEPOSITCONTROLSTATE.BASKET_SLIDE_EXTEND;
                }
                if (gamepad_1.getButton(GamepadKeys.Button.Y)) {
                    depositControlState = DEPOSITCONTROLSTATE.PICK_UP_ARM_EXTEND;
                }
                break;
            case BASKET_SLIDE_EXTEND:
                //check if
                //Color.RGBToHSV(robot.Color_Sensor.red() * 8, robot.Color_Sensor.green() * 8, robot.Color_Sensor.blue() * 8, RobotActionConfig.hsvValues);
                // extend slides to basket pos.
                //((RobotActionConfig.hsvValues[0] < 77 && RobotActionConfig.hsvValues[0] > 73)
                        //|| (RobotActionConfig.hsvValues[0] < 20 && RobotActionConfig.hsvValues[0]>16)
                        //|| (RobotActionConfig.hsvValues[0]<228 && RobotActionConfig.hsvValues[0]>224) && RobotActionConfig.hsvValues[1] > 0.5)
                    debounceTimer.reset();
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    liftTimer.reset();
                    depositControlState = DEPOSITCONTROLSTATE.BASKET_ARM_EXTEND;
                break;
                // extend arm
            case BASKET_ARM_EXTEND:
                if (isSlideAtPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos)) {
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_dump_Pos);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_dump_Pos);
                    depositControlState = DEPOSITCONTROLSTATE.BASKET_DUMP;
                }
                break;
            case BASKET_DUMP:
                if(servo_AtPosition(RobotActionConfig.deposit_Wrist_dump_Pos) || liftTimer.seconds() > 2) {
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    while(liftTimer.seconds() < 1){
                    }
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer_Pos);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer_Pos);
                    depositControlState = DEPOSITCONTROLSTATE.BASKET_RETRACT;
                }
                break;
            case BASKET_RETRACT:
                if(servo_AtPosition(RobotActionConfig.deposit_Arm_Transfer_Pos)) {
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    depositControlState = DEPOSITCONTROLSTATE.START;
                }
                break;
            case PICK_UP_ARM_EXTEND:
                //Color.RGBToHSV(robot.Color_Sensor.red() * 8, robot.Color_Sensor.green() * 8, robot.Color_Sensor.blue() * 8, RobotActionConfig.hsvValues);
                //&& ((RobotActionConfig.hsvValues[0] < 20 && RobotActionConfig.hsvValues[0] > 16)
                        //|| (RobotActionConfig.hsvValues[0] < 228 && RobotActionConfig.hsvValues[0] > 224) && RobotActionConfig.hsvValues[1] > 0.5)
                    debounceTimer.reset();
                    liftTimer.reset();
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_PickUp_Pos);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_PickUp_Pos);
                    depositControlState = DEPOSITCONTROLSTATE.PICK_UP_DROP;

                break;
            case PICK_UP_DROP:
                if(liftTimer.seconds()>2){
                    liftTimer.reset();
                    robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
                    if(servo_AtPosition(RobotActionConfig.deposit_Claw_Close) && liftTimer.seconds() > 3){
                        depositControlState = DEPOSITCONTROLSTATE.BAR_SLIDE_EXTEND;
                    }
                }
                break;
            case BAR_SLIDE_EXTEND:
                //Color.RGBToHSV(robot.Color_Sensor.red() * 8, robot.Color_Sensor.green() * 8, robot.Color_Sensor.blue() * 8, RobotActionConfig.hsvValues);
                if (  servo_AtPosition(RobotActionConfig.deposit_Claw_Close)&& gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER)&& debounceTimer.seconds()>DEBOUNCE_THRESHOLD
                        //(RobotActionConfig.hsvValues[2] < 35) &&
                    ){
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Up_Pos);
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Up_Pos);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
                    depositControlState = DEPOSITCONTROLSTATE.BASKET_ARM_EXTEND;
                }
                break;
            case BAR_ARM_EXTEND:
                if(isSlideAtPosition(RobotActionConfig.deposit_Slide_Highbar_Up_Pos)){
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Highbar_Pos);
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Highbar_Pos);
                    depositControlState = DEPOSITCONTROLSTATE.BAR_SCORE;
                }
                break;
            case BAR_SCORE:
                if(servo_AtPosition(RobotActionConfig.deposit_Wrist_Highbar_Pos)&& depositControlState == DEPOSITCONTROLSTATE.BAR_SCORE){
                    //int slides_Current_Position = robot.liftMotorLeft.getCurrentPosition();
                    if(gamepad_1.getButton(GamepadKeys.Button.DPAD_UP)) {
                        robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Up_Pos);
                        robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbar_Up_Pos);
                        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                        robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    }
                    if(gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN)){
                        robot.liftMotorRight.setTargetPosition(robot.liftMotorLeft.getCurrentPosition() - 100);
                        robot.liftMotorLeft.setTargetPosition(robot.liftMotorRight.getTargetPosition() - 100);
                        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                        robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    }
                    if(servo_AtPosition(RobotActionConfig.deposit_Claw_Open)){
                        depositControlState = DEPOSITCONTROLSTATE.BASKET_RETRACT;
                    }
                }
                break;
            case BAR_RETRACT:
                if (servo_AtPosition(RobotActionConfig.deposit_Claw_Open)){
                    robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer_Pos);
                    robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer_Pos);
                    while(liftTimer.seconds()<2){
                    }
                    robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
                    robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_down_Pos);
                    robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_DownLiftPower);
                    depositControlState = DEPOSITCONTROLSTATE.START;
                }
            default:
                depositControlState = DEPOSITCONTROLSTATE.START;
                break;
        }

        // Handle lift Cancel Action if 'B' button is pressed
        if ((gamepad_1.getButton(GamepadKeys.Button.B) || gamepad_2.getButton(GamepadKeys.Button.B)) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD && depositControlState != DEPOSITCONTROLSTATE.START) {
            debounceTimer.reset();
            depositControlState = DEPOSITCONTROLSTATE.START;
            robot.liftMotorLeft.setPower(0); // Ensure the motor is stopped
            robot.liftMotorRight.setPower(0);
            robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer_Pos);
            robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer_Pos);
            robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
            debounceTimer.reset();
            robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_hang_Pos);
            robot.liftMotorLeft.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
            robot.liftMotorRight.setTargetPosition(RobotActionConfig.deposit_Slide_Highbasket_Pos);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
            robot.liftMotorRight.setPower(RobotActionConfig.deposit_Slide_UpLiftPower);
        }

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN)&&gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && debounceTimer.seconds() > DEBOUNCE_THRESHOLD){
            debounceTimer.reset();
            int slides_Current_Position = robot.liftMotorLeft.getCurrentPosition();
            robot.liftMotorLeft.setTargetPosition(slides_Current_Position - 300);
            robot.liftMotorRight.setTargetPosition(slides_Current_Position - 300);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotorLeft.setPower(0.5);
            robot.liftMotorRight.setPower(0.5);
            if(gamepad_1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN) && gamepad_1.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
                robot.liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }
        }

        // Claw control - Button Back
        if((gamepad_1.getButton(GamepadKeys.Button.A) || gamepad_2.getButton(GamepadKeys.Button.A))&& debounceTimer.seconds() > DEBOUNCE_THRESHOLD + 0.25){
            debounceTimer.reset();
            ToggleDeposit();
            if (depositState == DEPOSITCLAWSTATE.OPEN) {
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
            } else {
                robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);
            }
        }
    }


    // Helper method to check if the lift is within the desired position threshold
    private boolean isSlideAtPosition(int targetPosition) {
        return Math.abs(robot.liftMotorLeft.getCurrentPosition() - targetPosition) < 5 && Math.abs(robot.liftMotorRight.getCurrentPosition() - targetPosition) < 5;
    }

    private boolean servo_AtPosition(double servoClawPosition) {
        return Math.abs(robot.depositClawServo.getPosition() - servoClawPosition) < 0.01;
    }

    DEPOSITCONTROLSTATE depositControlState(){
        return depositControlState;
    }
    //Deposit Claw depositControlState
    public enum DEPOSITCLAWSTATE {
        OPEN,
        CLOSE
    }
    //Toggle Deposit Open - Close
    private void ToggleDeposit() {
        if (depositState == DEPOSITCLAWSTATE.OPEN) {
            depositState = DEPOSITCLAWSTATE.CLOSE;
        } else {
            depositState = DEPOSITCLAWSTATE.OPEN;
        }
    }
}
