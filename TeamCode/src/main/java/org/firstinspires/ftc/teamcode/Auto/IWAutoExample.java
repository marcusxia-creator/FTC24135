package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.IceWaddlerPaths.ExamplePath;
import org.firstinspires.ftc.teamcode.IceWaddler;
import org.firstinspires.ftc.teamcode.TeleOps.FiniteStateMachineDeposit;
import org.firstinspires.ftc.teamcode.TeleOps.FiniteStateMachineIntake;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

/** config
 * deposit Left Arm Position initial position for installation = 0
 * deposit Left Arm Position initial position = 0
 *
 */



@Autonomous(name = "IceWaddlerAuto", group = "org.firstinspires.ftc.teamcode.Auto")
public class IWAutoExample extends OpMode {

    //Robot
    public RobotHardware robot;                            //For robot drive

    public GamepadEx gamepadCo1;                            //For gamepad
    public GamepadEx gamepadCo2;

    //Robot Intake & Deposit
    public FiniteStateMachineDeposit depositArmDrive;       //For Robot Deposit Arm
    public FiniteStateMachineIntake intakeArmDrive;         //For Robot Intake

    public IceWaddler iceWaddler;

    FtcDashboard dashboard;

    @Override
    public void init() {

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware in RobotHardware
        robot = new RobotHardware();
        robot.init(hardwareMap);                                                   // Initialize RobotDrive

        //gamepad
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        //Deposit Arm control
        depositArmDrive = new FiniteStateMachineDeposit(robot, gamepadCo1, gamepadCo2, intakeArmDrive, telemetry); // Pass parameters as needed);

        //Intake Arm Control
        intakeArmDrive = new FiniteStateMachineIntake(robot, gamepadCo1,gamepadCo2, depositArmDrive);
        intakeArmDrive.Init();

        iceWaddler = new IceWaddler(robot);
        iceWaddler.Init(IceWaddler.CONTROLMODE.VELOCITY, ExamplePath.startingPose, true);

        //Start Path
        iceWaddler.runPath(ExamplePath.Path);

        //Telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addLine("-------------------");
        telemetry.addData("Status", " initialized Motors and Encoder and IMU and Arm Control");
        telemetry.addData("Control Mode", iceWaddler.controlMode.name());
        telemetry.addLine("-------------------");
        telemetry.addData("Vertical slide Encoder_left",robot.liftMotorLeft.getCurrentPosition());
        telemetry.addData("Vertical slide Encoder_right",robot.liftMotorRight.getCurrentPosition());

        telemetry.update();
        resetRuntime();
        }

    @Override
    public void loop () {
        // when position is back to 0, deposit is initialized.
        //depositArmDrive.Init();

        iceWaddler.loop();

        intakeArmDrive.IntakeArmLoop();
        depositArmDrive.DepositArmLoop();

        // Telemetry
        telemetry.addData("Run Mode", iceWaddler.controlMode.name());

        telemetry.addLine("---------------------");
        telemetry.addData("Deposit Arm Position", robot.depositArmServo.getPosition());
        telemetry.addData("Deposit Wrist Position", robot.depositWristServo.getPosition());
        telemetry.addData("Deposit Claw Position", robot.depositClawServo.getPosition());

        telemetry.addLine("---------------------");
        telemetry.addData("Intake Arm Left Position", robot.intakeLeftArmServo.getPosition());
        telemetry.addData("Intake Arm Right Position", robot.intakeRightArmServo.getPosition());
        telemetry.addData("Intake Wrist Position", robot.intakeWristServo.getPosition());
        telemetry.addData("Intake Claw Position", robot.intakeClawServo.getPosition());
        telemetry.addData("Intake Slide LEFT Position", robot.intakeLeftSlideServo.getPosition());
        telemetry.addData("Intake Slide RIGHT Position", robot.intakeRightSlideServo.getPosition());

        telemetry.addLine("---------------------");
        telemetry.addData("Limit Switch Pressed", robot.limitSwitch.getState());
        telemetry.addLine("---------------------");
        telemetry.addData("Pinpoint Pose", iceWaddler.currentPos);
        /*
        telemetry.addLine("---------Frequency--------");
        telemetry.addData("Pinpoint Frequency", drive.pinPointFrequency()); //prints/gets the current refresh rate of the Pinpoint
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
         */
        telemetry.update();

        //Actions
        if(iceWaddler.currentActionIndex==0 && intakeArmDrive.intakeState == FiniteStateMachineIntake.INTAKESTATE.INTAKE_START && iceWaddler.distanceRemaining<=ExamplePath.intakeExtendDistance){
            //Extend Intake
            robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Idle);
            robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Idle);
            robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Idle);
            // reset time for next step
            intakeArmDrive.intakeTimer.reset();
            intakeArmDrive.intakeState = FiniteStateMachineIntake.INTAKESTATE.INTAKE_EXTEND;
        }

        if(iceWaddler.currentActionIndex==2 && intakeArmDrive.intakeState==FiniteStateMachineIntake.INTAKESTATE.INTAKE_PICK){
            //Pickup Sample
            robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Left_Grab);
            robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Right_Grab);
            //retract
            intakeArmDrive.intakeTimer.reset();
            intakeArmDrive.intakeState = FiniteStateMachineIntake.INTAKESTATE.INTAKE_SAMPLE_RETRACT;
        }

        if(iceWaddler.currentActionIndex==3 && intakeArmDrive.intakeState==FiniteStateMachineIntake.INTAKESTATE.INTAKE_START && depositArmDrive.liftState == FiniteStateMachineDeposit.LIFTSTATE.LIFT_START){
            depositArmDrive.liftState = FiniteStateMachineDeposit.LIFTSTATE.LIFT_HIGHBASKET;
            depositArmDrive.liftTimer.reset();
            depositArmDrive.liftUpTimeout.reset();
        }

        if(iceWaddler.currentActionIndex==4 && depositArmDrive.liftState == FiniteStateMachineDeposit.LIFTSTATE.LIFT_RETRACT){
            iceWaddler.actionCompleted=true;
        }
    }

    //Stop the Robot
    public void stop () {
        iceWaddler.zeroPower();
        robot.liftMotorLeft.setPower(0);
        robot.liftMotorRight.setPower(0);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }
}
