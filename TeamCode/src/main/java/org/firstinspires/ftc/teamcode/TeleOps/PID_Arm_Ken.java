package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.controller.PIDFController;
@TeleOp
public class PID_Arm_Ken extends OpMode{
    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private PIDFController pidfController = new PIDFController (0, 0, 0, 0);
    RobotHardware robot;

    public PID_Arm_Ken(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
        this.robot = robot;

    }

    @Override
    public void init (){
        pidfController = new PIDFController(0, 0 ,0, 0);
    }

    public void loop (){
        pidfController.setPIDF(0,0,0,0);
        int slidePos = robot.liftMotorLeft.getCurrentPosition();

    }
}


