package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleOpAxonServoTest extends OpMode {

    public Servo servo;
    public AnalogInput pot;
    public double servoAngle;

    public void init() {
        servo = hardwareMap.get(Servo.class, "Intake_Wrist_Servo");
        pot = hardwareMap.get(AnalogInput.class, "pot");
        servo.setPosition(0);
    }
    public void loop() {
        if (gamepad1.dpad_up) {
            servo.setPosition(Range.clip((servo.getPosition() + 0.01), 0, 1));
        }
        if (gamepad1.dpad_down) {
            servo.setPosition(Range.clip((servo.getPosition() - 0.01), 0, 1));
        }
        if (gamepad1.a) {
            servo.setPosition(1);
        }
        if (gamepad1.b) {
            servo.setPosition(0);
        }
        if (gamepad1.x) {
            servo.setPosition(0.5);
        }
        if (gamepad1.y) {
            servo.setPosition(0.25);
        }


        double servoAngle =  getPotAngle();
        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.addData("Servo Position ANGLE", servoAngle);
        telemetry.addData("Servo Max Voltage", pot.getMaxVoltage());
        telemetry.update();
    }
    public double getPotAngle(){
        return Range.scale(pot.getVoltage(),0,pot.getMaxVoltage(),0,180);
    }
}
