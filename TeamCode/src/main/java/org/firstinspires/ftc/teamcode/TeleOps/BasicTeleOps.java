package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkData;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.drive.StandardTrackingWheelLocalizer;

import java.util.List;

@Disabled
@Config
@TeleOp(name = "TeleOps_Premier", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOps extends OpMode {

    public enum ControlState { RUN, TEST }

    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private FiniteStateMachineDeposit depositArmDrive;
    private FiniteStateMachineIntake intakeArmDrive;
    private ServoTest servoTest;
    private ControlState controlState = ControlState.RUN;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private boolean lBstartPressed = false;
    private List<LynxModule> allHubs;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        depositArmDrive = new FiniteStateMachineDeposit(robot, gamepadCo1, gamepadCo2, intakeArmDrive, telemetry);


        intakeArmDrive = new FiniteStateMachineIntake(robot, gamepadCo1, gamepadCo2, depositArmDrive);
        intakeArmDrive.Init();

        servoTest = new ServoTest(robot, gamepadCo1, gamepadCo2);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        double batteryVolts_ini = getBatteryVoltage();

        telemetry.addLine("-------------------");
        telemetry.addData("Status", "initialized");
        telemetry.addData("Battery Volt", batteryVolts_ini);
        telemetry.addData("Control Mode", robotDrive.getDriveMode().name());
        telemetry.addData("VS Left Encoder", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("VS Right Encoder", robot.liftMotorRight.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepadCo1.getButton(BACK) && !gamepadCo1.getButton(LEFT_BUMPER) && isButtonDebounced()) {
            debounceTimer.reset();
            robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.liftMotorLeft.setPower(-0.3);
            robot.liftMotorRight.setPower(-0.3);
            //resetLiftEncoders();
            //depositArmDrive.Init();
            //depositArmDrive.liftMotorInit();
        }

        /// FORCE SLIDE GO ALL THE WAY DOwn.
         if (gamepadCo1.getButton(BACK) && gamepadCo1.getButton(LEFT_BUMPER) && isButtonDebounced())
         { /**
             int resetPositionL = Math.abs(robot.liftMotorLeft.getCurrentPosition());
             int resetPositionR = Math.abs(robot.liftMotorRight.getCurrentPosition());
             int resetPosition = (resetPositionL+resetPositionR)/2;
             if (resetPosition!=0) {
                 robot.liftMotorLeft.setTargetPosition(-resetPosition);
                 robot.liftMotorRight.setTargetPosition(-resetPosition);
                 robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.liftMotorLeft.setPower(0.3);
                 robot.liftMotorRight.setPower(0.3);
             } else {
                 robot.liftMotorLeft.setTargetPosition(-4000);
                 robot.liftMotorRight.setTargetPosition(-4000);
                 robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.liftMotorLeft.setPower(0.3);
                 robot.liftMotorRight.setPower(0.3);
                 while (robot.liftMotorLeft.isBusy()||robot.liftMotorRight.isBusy()){
                     if(gamepadCo2.getButton(BACK)&&gamepadCo2.getButton(LEFT_BUMPER)&& isButtonDebounced()){
                         //Stop motor
                         robot.liftMotorLeft.setPower(0.0);
                         robot.liftMotorRight.setPower(0.0);
                         // 3) Now that we're at zero, actually reset the encoder count
                         robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                         robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                         // 4) Return to a normal run mode if you still want to drive by power/velocity
                         robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                         robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                         break;
                     }
                 }
             }
          */
             robot.liftMotorLeft.setPower(0.0);
             robot.liftMotorRight.setPower(0.0);
             // 3) Now that we're at zero, actually reset the encoder count
             robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             // 4) Return to a normal run mode if you still want to drive by power/velocity
             robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             depositArmDrive.Init();
         }

        for (LynxModule hub : allHubs) {
            BulkData bulkData = hub.getBulkData();
            if (bulkData != null) {
                if (hub.equals(allHubs.get(0))) {
                    telemetry.addData("Drive FL Pos", bulkData.getMotorCurrentPosition(robot.frontLeftMotor.getPortNumber()));
                    telemetry.addData("Drive FR Pos", bulkData.getMotorCurrentPosition(robot.frontRightMotor.getPortNumber()));
                    telemetry.addData("Drive BL Pos", bulkData.getMotorCurrentPosition(robot.backLeftMotor.getPortNumber()));
                    telemetry.addData("Drive BR Pos", bulkData.getMotorCurrentPosition(robot.backRightMotor.getPortNumber()));
                } else {
                    telemetry.addData("Lift L Pos", bulkData.getMotorCurrentPosition(robot.liftMotorLeft.getPortNumber()));
                    telemetry.addData("Lift R Pos", bulkData.getMotorCurrentPosition(robot.liftMotorRight.getPortNumber()));
                }
            }
        }

        robotDrive.DriveLoop();

        if (gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed) {
            toggleControlState();
            debounceTimer.reset();
            lBstartPressed = true;
        } else if (!gamepadCo1.getButton(START) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }

        if (controlState == ControlState.RUN) {
            depositArmDrive.DepositArmLoop();
            intakeArmDrive.IntakeArmLoop();

            telemetry.addData("Deposit State", depositArmDrive.liftState);
            telemetry.addData("Deposit Claw State", depositArmDrive.depositClawState);
            telemetry.addData("Intake State", intakeArmDrive.intakeState);
            telemetry.addData("Intake Claw State", intakeArmDrive.intakeClawState);
            /**telemetry.addData("Color Hue", RobotActionConfig.hsvValues[0]);
             telemetry.addData("Color Value", RobotActionConfig.hsvValues[2]); **/
        }

        if (controlState == ControlState.TEST) {
            servoTest.loop();
        }


        double batteryVolts = getBatteryVoltage();
        telemetry.addData("Battery (V)", "%.2f", batteryVolts);
        telemetry.addLine("-----Drive-----");
        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Drive Mode", robotDrive.getDriveMode().name());
        telemetry.addData("Heading", robot.imu.getRobotYawPitchRollAngles().getYaw());
        // --- Telemetry ---

        telemetry.addLine("-----Deposit-----");
        telemetry.addData("VS Left mm", (double) robot.liftMotorLeft.getCurrentPosition() / RobotActionConfig.TICKS_PER_MM_SLIDES);
        telemetry.addData("VS Right mm", (double) robot.liftMotorRight.getCurrentPosition() / RobotActionConfig.TICKS_PER_MM_SLIDES);
        telemetry.addData("VS Left tick", robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("VS Right tick", robot.liftMotorRight.getCurrentPosition());
        telemetry.addData("Deposit Left Arm Position", robot.depositLeftArmServo.getPosition());
        telemetry.addData("Deposit Right Arm Position", robot.depositRightArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw Position", robot.depositClawServo.getPosition());
        telemetry.addLine("-----Intake-----");
        telemetry.addData("Intake Arm Position", robot.intakeArmServo.getPosition());
        telemetry.addData("Intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addData("Intake Slide Left Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide Right Position", robot.intakeRightSlideServo.getPosition());
        telemetry.addData("Intake Turret Position", robot.intakeTurretServo.getPosition());
        telemetry.addData("Intake Rotation Position", robot.intakeRotationServo.getPosition());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }

    private void toggleControlState() {
        controlState = (controlState == ControlState.RUN) ? ControlState.TEST : ControlState.RUN;
    }

    /**
    private boolean LSisPressed() {
        return robot.limitSwitch.getState();
    }
     */

    private void resetLiftEncoders()
    {
        /**
         *         robot.liftMotorLeft.setTargetPosition(-4000);
         *         robot.liftMotorRight.setTargetPosition(-4000);
         *         robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         *         robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */

        robot.liftMotorLeft.setPower(-0.3);
        robot.liftMotorRight.setPower(-0.3);
        /**
        while (
                (robot.liftMotorLeft.isBusy()||robot.liftMotorRight.isBusy())) {
            if (gamepadCo1.getButton(BACK) && gamepadCo1.getButton(LEFT_BUMPER) && isButtonDebounced()) {
                break;
            }
        }
         */
        //Stop motor

    }
    // Debouncer helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    ///add a voltage sensor
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && voltage < result) {
                result = voltage;
            }
        }
        // If we never found a positive reading, default to 0
        return (result == Double.POSITIVE_INFINITY) ? 0.0 : result;
    }

}
