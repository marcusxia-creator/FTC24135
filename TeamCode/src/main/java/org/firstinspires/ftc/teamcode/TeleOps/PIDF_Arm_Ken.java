package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@TeleOp
public class PIDF_Arm_Ken extends OpMode{
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private PIDController pidController = new PIDController (0, 0, 0);
    private double f = 0;
    private int target = 0;
    RobotHardware robot;
    public PIDF_Arm_Ken(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;

    }

    @Override
    public void init (){
        pidController = new PIDController(0, 0 ,0);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
    }

    public void loop (){
        pidController.setPID(0,0,0);
        int slidePos = robot.liftMotorLeft.getCurrentPosition();
        double pid = pidController.calculate(slidePos, target);
        double ff = 0;
        telemetry.addData("target", target);
        telemetry.addData("Lift Motor Left", robot.liftMotorLeft.getCurrentPosition());


    }

}


