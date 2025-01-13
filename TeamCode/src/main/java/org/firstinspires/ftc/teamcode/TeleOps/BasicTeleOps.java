package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/** config
 * deposit Left Arm Position initial position for installation = 0
 * deposit Left Arm Position initial position = 0
 *
 */

@Config
@TeleOp(name = "TeleOps_MW_FMS_v2.2_Competition", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {
    //Robot
    public RobotHardware robot;                     // Bring in robot hardware configuration
    public GamepadEx gamepadCo1;                    //For gamepad
    public GamepadEx gamepadCo2;
    public Gamepad gamepad_1;
    public Gamepad gamepad_2;


    //Robot drive
    public RobotDrive robotDrive;                   //For robot drive

    //Robot Intake & Deposit
    public RobotDeposit depositArmDrive;   //For Robot Arm
    public RobotIntake intakeArmDrive; //For Robot Intake

    public ServoTest servoTest;

    private ControlState controlState = ControlState.RUN;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    private boolean startPressed = false;

    float hsvValues[] = {0F,0F,0F};

    //Bulk Reading
    private List<LynxModule> allHubs;

    
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware in RobotHardware
        robot = new RobotHardware();
        robot.init(hardwareMap);

        //robot configuration

        //gamepad
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        //robotDrive
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);   // Pass robot instance to RobotDrive
        robotDrive.Init();                                                              // Initialize RobotDrive

        //Deposit Arm control
        depositArmDrive = new RobotDeposit(robot, gamepadCo1, gamepadCo2); // Pass parameters as needed);
        depositArmDrive.Init();

        //Intake Arm Control
        intakeArmDrive = new RobotIntake(robot, gamepadCo1, gamepadCo2, gamepad1, gamepad2);
        intakeArmDrive.intakeInit();

        //Servo Testing
        servoTest = new ServoTest(robot, gamepadCo1, gamepadCo2);
        //servoTest.ServoTestInit();


        // get bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //Robot Control State
        RobotDrive.DriveMode currentDriveMode = robotDrive.getDriveMode();

        //
        telemetry.addLine("-------------------");
        telemetry.addData("Status", " initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addData("Control Mode", currentDriveMode.name());
        telemetry.addLine("-------------------");
        }

    @Override
    public void loop () {

        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                // Example: Reading motor position for each hub
                if (hub.equals(allHubs.get(0))) { // Assuming the first hub is Control Hub
                    int frontLeftMotor = bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber());
                    int frontRightMotor = bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber());

                    telemetry.addData("Drive Motor FL Motor (Control Hub) Position", frontLeftMotor);
                    telemetry.addData("Drive Motor FR Motor (Control Hub) Position", frontRightMotor);
                } else if (hub.equals(allHubs.get(1))) { // Assuming the second hub is Expansion Hub
                    int liftLeftMotor = bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber());
                    int  liftRightMotor= bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber());
                    telemetry.addData("Deposit Left Motor Position (Expansion Hub)", liftLeftMotor);
                    telemetry.addData("Deposit right Motor Position (Expansion Hub)", liftRightMotor);
                }
            }
        }


        robotDrive.DriveLoop(); // Use RobotDrive methods
        RobotDrive.DriveMode currentDriveMode = robotDrive.getDriveMode();

        if ((gamepadCo1.getButton(BACK) && gamepadCo1.getButton(LEFT_BUMPER)) && !startPressed) {
            toggleControlState();
            debounceTimer.reset();
            startPressed = true;
        } else if (!(gamepadCo1.getButton(BACK) && gamepadCo1.getButton(LEFT_BUMPER))) {
            startPressed = false;
        }

        if (controlState == ControlState.RUN) {
            depositArmDrive.DepositControl();
            RobotDeposit.DEPOSITCONTROLSTATE liftState = depositArmDrive.depositControlState();
            intakeArmDrive.intakeSlideControl();
            RobotIntake.IntakeState intakeState = intakeArmDrive.intakeState();
            telemetry.addLine("---------------------");
            telemetry.addData("Deposit State", liftState.name());
            telemetry.addData("Intake State", intakeState.name());
            telemetry.addLine("---------------------");
            //telemetry.addData("Color",depositArmDrive.Color());
        } else {
            servoTest.ServoTestLoop();
        }

        // Telemetry
        telemetry.addLine("---------------------");
        telemetry.addData("Deposit Arm Position", robot.depositArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("Intake Arm Left Position", robot.intakeLeftArmServo.getPosition());
        telemetry.addData("Intake Arm Right Position", robot.intakeRightArmServo.getPosition());
        telemetry.addData("Intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Slide Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("intake Rotation Position", robot.intakeRotationServo.getPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("deposit Claw Position", robot.depositClawServo.getPosition());
        telemetry.addData("intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("Left Slide Position", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("Right Slide Position", robot.liftMotorRight.getCurrentPosition());
        telemetry.addLine("---------------------");
        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Drive Mode", currentDriveMode.name());
        telemetry.addLine("---------------------");
        telemetry.addData("Heading ", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Hue detected", RobotActionConfig.hsvValues[0]);
        telemetry.addData("Value detected", RobotActionConfig.hsvValues[2]);
        telemetry.addData("Saturation detected", RobotActionConfig.hsvValues[1]);
        telemetry.update();
    }


    public void stop () {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        //robot.IntakeServo.setPosition(1.0);
        telemetry.addData("Status", "Robot stopped");
    }
    public enum ControlState {
        RUN,
        TEST
    }

    private void toggleControlState () {
        if (controlState != ControlState.RUN) {
            controlState = ControlState.RUN;
        } else {
            controlState = ControlState.TEST;
        }
    }

}
