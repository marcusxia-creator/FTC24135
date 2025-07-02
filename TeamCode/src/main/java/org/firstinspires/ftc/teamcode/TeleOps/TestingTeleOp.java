package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestingTeleOp extends OpMode {
    //private AutoVisionProcessing autoVisionProcessing;
    private FtcDashboard dashboard;

    FiniteStateMachineVision finiteStateMachineVision;
    RobotHardware robot;
    FiniteStateMachineIntake intakeArmDrive;
    FiniteStateMachineDeposit depositArmDrive;

    GamepadEx gamepadCo1;
    GamepadEx gamepadCo2;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //autoVisionProcessing = new AutoVisionProcessing(dashboard ,hardwareMap);
        //autoVisionProcessing.initialize();

        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);

        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        depositArmDrive = new FiniteStateMachineDeposit(robot, gamepadCo1, gamepadCo2, intakeArmDrive, telemetry);
        depositArmDrive.Init();

        intakeArmDrive = new FiniteStateMachineIntake(robot, gamepadCo1, gamepadCo2, depositArmDrive);
        intakeArmDrive.Init();

        finiteStateMachineVision = new FiniteStateMachineVision(dashboard, robot, hardwareMap, gamepadCo1, gamepadCo2, intakeArmDrive, false);
        finiteStateMachineVision.init(false, false, true);
    }

    @Override
    public void loop() {

        /*
        autoVisionProcessing.process(false);

        if (autoVisionProcessing.sampleX != 0.0) {
            telemetry.addData("Real X", autoVisionProcessing.sampleX);
            telemetry.addData("Real Y", autoVisionProcessing.sampleY);
            telemetry.addData("Angles", autoVisionProcessing.sampleAngles);
        } else {
            telemetry.addLine("No Sample Detected");
        }
    }
         */

        finiteStateMachineVision.visionLoop(false);
        /**
        if (finiteStateMachineVision.bestSample.relPos != null) {
            telemetry.addData("Sample Pose", finiteStateMachineVision.bestSample.relPos);
        }
         */
        telemetry.addData("States", finiteStateMachineVision.visionState);
        telemetry.addData("Sample Angles", finiteStateMachineVision.pipeline.angles);
        telemetry.update();
    }
}
