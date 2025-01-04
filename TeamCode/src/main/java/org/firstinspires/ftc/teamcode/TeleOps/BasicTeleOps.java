package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/** config
 * deposit Left Arm Position initial position for installation = 0
 * deposit Left Arm Position initial position = 0
 *
 */

@Config
@TeleOp(name = "TeleOps_MW_FMS_v2.0", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {
    //Robot
    public RobotHardware robot;                     // Bring in robot hardware configuration

    public GamepadEx gamepadCo1;                    //For gamepad
    public GamepadEx gamepadCo2;

    //Robot drive
    public RobotDrive robotDrive;//For robot drive

    private ElapsedTime debounceTimer = new ElapsedTime();

    //Robot Intake & Deposit
    public FiniteStateMachineArm depositControlDrive;//For Robot Arm
    public RobotIntake intakeDrive;

    public ServoTest servoTest;

    private TESTSTATE testState;


    //Bulk Reading
    private List<LynxModule> allHubs;


    //Drive power factor
    //public static double powerFactor = 0.5;


    
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware in RobotHardware
        robot = new RobotHardware();
        robot.init(hardwareMap);

        //robot configuration

        //gamepad
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        //robotDrive
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);   // Pass robot instance to RobotDrive
        robotDrive.Init();// Initialize RobotDrive

        servoTest = new ServoTest(robot, gamepadCo1, gamepadCo2);
        servoTest.ServoTestInit();


        //Deposit Arm control
        depositControlDrive = new FiniteStateMachineArm(robot, gamepadCo1,gamepadCo2); // Pass parameters as needed);
        depositControlDrive.Init();

        //Intake Arm Control
        intakeDrive = new RobotIntake(robot, gamepadCo2,gamepadCo1);
        intakeDrive.intakeInit();


        // get bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //Robot Control depositControlState
        RobotDrive.ControlMode currentMode = robotDrive.getControlMode();

        //
        telemetry.addLine("-------------------");
        telemetry.addData("Status"," initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addData("Control Mode", currentMode.name());
        telemetry.addLine("-------------------");
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                // Example: Reading motor position for each hub
                if (hub.equals(allHubs.get(0))) { // Assuming the first hub is Control Hub
                    int frontLeftMotor = bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber());
                    int frontRightMotor = bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber());

                    telemetry.addData("Deposit Left Motor Position (Expansion Hub)", frontLeftMotor);
                    telemetry.addData("Deposit right Motor Position", frontRightMotor);
                } else if (hub.equals(allHubs.get(1))) { // Assuming the second hub is Expansion Hub
                    int liftMotorLeft = bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber());
                    int liftMotorRight = bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber());
                    telemetry.addData("Drive Motor FL Motor (Control Hub) Position", liftMotorLeft);
                    telemetry.addData("Drive Motor FR Motor (Control Hub) Position", liftMotorRight);
                }
            }

        }

        robotDrive.DriveLoop(); // Use RobotDrive methods
        RobotDrive.ControlMode currentMode = robotDrive.getControlMode();

        if((gamepadCo1.getButton(GamepadKeys.Button.BACK) && gamepadCo1.getButton(GamepadKeys.Button.LEFT_BUMPER)
                || gamepadCo2.getButton(GamepadKeys.Button.BACK) && gamepadCo2.getButton(GamepadKeys.Button.LEFT_BUMPER))&&
                debounceTimer.seconds()>RobotActionConfig.DEBOUNCE_THRESHOLD){
            debounceTimer.reset();
            ToggleTest();
        }


        if (testState != TESTSTATE.TEST) {
            depositControlDrive.DepositControl();
            intakeDrive.intakeSlideControl();
            FiniteStateMachineArm.DEPOSITCONTROLSTATE depositControlState = depositControlDrive.depositControlState();
            RobotIntake.IntakeState intakeState = intakeDrive.intakeState();
        } else {
            servoTest.ServoTestLoop();
        }


        // Telemetry
        telemetry.addData("deposit Left Arm Position", robot.depositArmServo.getPosition());
        telemetry.addData("deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("deposit Claw Position", robot.depositClawServo.getPosition());
        telemetry.addData("intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addData("intake Left Arm Position", robot.intakeLeftArmServo.getPosition());
        telemetry.addData("intake Right Arm Position", robot.intakeRightArmServo.getPosition());
        telemetry.addData("intake Rotation Position", robot.intakeRotationServo.getPosition());
        telemetry.addData("intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Control Mode", currentMode.name());
        telemetry.addData("Left Slide Position", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("Right Slide Position", robot.liftMotorRight.getCurrentPosition());
        telemetry.addData("Heading ", robot.imu.getRobotYawPitchRollAngles().getYaw());
        //telemetry.addData("Deposit State", depositControlState.name());
       // telemetry.addData("Bar State", barState.name());
       // telemetry.addData("Intake State", intakeState.name());
        telemetry.addData("Colour detected", RobotActionConfig.hsvValues[0]);
        if (testState != TESTSTATE.TEST) {
            telemetry.addData("State", "RUN");
            telemetry.addData("Deposit Control", "Active");
            telemetry.addData("Intake Slide Control", "Active");
        } else {
            telemetry.addData("State", "SERVO TEST");
            telemetry.addData("Servo Test", "Active");
        }
        telemetry.update();
    }

    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        //robot.IntakeServo.setPosition(1.0);
        telemetry.addData("Status", "Robot stopped");
    }
    private void ToggleTest() {
        if (testState == TESTSTATE.TEST){
            testState = TESTSTATE.RUN;
        } else{
            testState = TESTSTATE.TEST;
        }
    }
    public enum TESTSTATE {
        TEST,
        RUN,
    }
}
