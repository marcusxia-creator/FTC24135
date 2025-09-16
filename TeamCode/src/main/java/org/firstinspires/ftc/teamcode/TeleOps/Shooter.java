package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Shooter_Prototype", group = "org.firstinspires.ftc.teamcode")
public class Shooter extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */
    // === Dashboard-tunable params ===
    public static double BASE_POWER_A = 0.10;  // power set by button A
    public static double STEP = 0.050;          // dpad increment size
    public static double MIN_POWER = 0.00;     // clamp lower bound
    public static double MAX_POWER = 1.00;     // clamp upper bound
    public static long   DEBOUNCE_MS = 180;    // dpad debounce

    // --- Motor variables ---
    private DcMotorEx motorLeft;
    private DcMotorEx motorRight;
    private double motorPower = 0.0;

    // --- Encoder constants (change for your motor) ---
    // REV HD Hex (eg. 312 RPM) uses 28 ticks/rev * gear ratio.
    // Example: Neverest 40 = 1120 ticks/rev, GoBilda 5202/5203 = 537.7 ticks/rev.
    // Example: gobilda 1150= 145.1 ticks/rev, GoBilda 5202/5203 = 537.7 ticks/rev.for 312rpm
    public static double TICKS_PER_REV = 28;   // change to your motor
    public static double RPM_CONVERSION = 60.0 / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motor
        motorLeft = hardwareMap.get(DcMotorEx.class, "MotorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "MotorRight");
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry to DS + Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready. A=0.10, B=stop, Dpad Up/Down=±0.10");
        telemetry.update();

        waitForStart();

        long lastStepTime = 0;

        while (opModeIsActive()) {
            // Button A → set to base power
            if (gamepad1.a) {
                motorPower = BASE_POWER_A;
            }

            // Button B → stop
            if (gamepad1.b) {
                motorPower = 0.0;
            }

            // Debounced Dpad steps
            long now = System.currentTimeMillis();
            if (gamepad1.dpad_up && now - lastStepTime >= DEBOUNCE_MS) {
                motorPower += STEP;
                lastStepTime = now;
            }
            if (gamepad1.dpad_down && now - lastStepTime >= DEBOUNCE_MS) {
                motorPower -= STEP;
                lastStepTime = now;
            }

            // Clamp
            motorPower = Math.max(MIN_POWER, Math.min(MAX_POWER, motorPower));

            // Apply power
            motorLeft.setPower(motorPower);
            motorRight.setPower(motorPower);

            // === Read velocity in ticks/sec and convert to RPM ===
            double ticksPerSec = motorLeft.getVelocity();
            double rpm = ticksPerSec * RPM_CONVERSION;

            double ticksPerSec2 = motorRight.getVelocity();
            double rpm2 = ticksPerSec2 * RPM_CONVERSION;

            // Telemetry (DS + Dashboard)
            telemetry.addData("Power", "%.3f", motorPower);
            telemetry.addData("Ticks/sec", "%.1f", ticksPerSec);
            telemetry.addData("RPM", "%.1f", rpm);

            telemetry.addData("Ticks/sec-right", "%.1f", ticksPerSec2);
            telemetry.addData("RPM-right", "%.1f", rpm2);
            telemetry.update();

            // Dashboard custom packet
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("power", motorPower);
            packet.put("rpm", rpm);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            idle();
        }
    }
}
