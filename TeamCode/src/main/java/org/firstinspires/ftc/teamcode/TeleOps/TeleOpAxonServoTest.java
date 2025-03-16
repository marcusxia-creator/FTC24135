package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleOpAxonServoTest extends OpMode {

    public Servo servo;

    public void init() {
        servo = hardwareMap.get(Servo.class, "Intake_Wrist_Servo");
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

        telemetry.addData("Servo Position", servo.getPosition());
        telemetry.update();
    }
}
