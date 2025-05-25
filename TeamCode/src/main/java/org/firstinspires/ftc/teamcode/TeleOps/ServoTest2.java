package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Four Servos Tester", group = "Testing")
public class ServoTest2 extends LinearOpMode {

    private Servo servogoBilda;
    private Servo servoAxonMax;
    private Servo servoAxonMini;
    private Servo servoSWFTY;

    private final double step = 0.05;
    private final long delayMillis = 200;

    @Override
    public void runOpMode() {
        // Update this name to match your configuration
        servoAxonMax = hardwareMap.get(Servo.class, "servoAxonMax");
        servogoBilda = hardwareMap.get(Servo.class, "servogoBilda");
        servoAxonMini = hardwareMap.get(Servo.class, "servoAxonMini");
        servoSWFTY = hardwareMap.get(Servo.class, "servoSWYFT");

        servogoBilda.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Press A for 0.0");
        telemetry.addLine("Press B for 0.5");
        telemetry.addLine("Press X for 1.0");
        telemetry.addLine("Press D-pad Up to sweep 0.0 -> 1.0");
        telemetry.addLine("Press D-Pad Down to sweep 1.0 -> 0.0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                servoAxonMax.setPosition(0.0);
                servogoBilda.setPosition(0.0);
                servoAxonMini.setPosition(0.0);
                servoSWFTY.setPosition(0.0);
                telemetry.addData("Set Position", "0.0");
                telemetry.update();
                sleep(300);
            }

            if (gamepad1.b) {
                servoAxonMax.setPosition(0.5);
                servogoBilda.setPosition(0.5);
                servoAxonMini.setPosition(0.5);
                servoSWFTY.setPosition(0.5);
                telemetry.addData("Set Position", "0.5");
                telemetry.update();
                sleep(300);
            }

            if (gamepad1.x) {
                servoAxonMax.setPosition(1.0);
                servogoBilda.setPosition(1.0);
                servoAxonMini.setPosition(1.0);
                servoSWFTY.setPosition(1.0);
                telemetry.addData("Set Position", "1.0");
                telemetry.update();
                sleep(300);
            }

            if (gamepad1.dpad_up) {
                for (double pos = 0.0; pos <= 1.0; pos += step) {
                    servoAxonMax.setPosition(pos);
                    servogoBilda.setPosition(pos);
                    servoAxonMini.setPosition(pos);
                    servoSWFTY.setPosition(pos);
                    telemetry.addData("Sweeping", "%.2f", pos);
                    telemetry.update();
                    sleep(delayMillis);
                }
            }

            if (gamepad1.dpad_down) {
                for (double pos = 1.0; pos >= 0.0; pos -= step) {
                    servoAxonMax.setPosition(pos);
                    servogoBilda.setPosition(pos);
                    servoAxonMini.setPosition(pos);
                    servoSWFTY.setPosition(pos);
                    telemetry.addData("Sweeping", "%.2f", pos);
                    telemetry.update();
                    sleep(delayMillis);
                }
            }

            idle();
        }
    }
}

