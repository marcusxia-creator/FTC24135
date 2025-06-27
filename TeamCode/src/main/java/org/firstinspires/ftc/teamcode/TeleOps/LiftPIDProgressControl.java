package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Lift PID Progress Control (OpMode)")
public class LiftPIDProgressControl extends OpMode {

    private DcMotor liftMotor_Left;
    private DcMotor liftMotor_Right;
    private PIDController pid;

    // Tunable values via FTC Dashboard
    public static double kP = 2.0;
    public static double kI = 0.0;
    public static double kD = 0.1;

    public static int upTarget = 500;
    public static int downTarget = 100;

    public static int startPosition = 0;
    public static int targetPosition = 0;
    private ElapsedTime debouncer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        liftMotor_Left = hardwareMap.get(DcMotor.class, "VS_Motor_Left");
        liftMotor_Right = hardwareMap.get(DcMotor.class, "VS_Motor_Right");

        for (DcMotor motor : new DcMotor[]{liftMotor_Left, liftMotor_Right}) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        liftMotor_Left.setDirection(DcMotorSimple.Direction.REVERSE);

        pid = new PIDController(kP, kI, kD);

        telemetry.addLine("Initialized. Press PLAY.");
        telemetry.addData("Left Motor Position.", liftMotor_Left.getCurrentPosition());
        telemetry.addData("Right Motor Position.", liftMotor_Right.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        // Allow tuning during runtime
        pid.setPID(kP, kI, kD);

        // Handle input
        if (gamepad1.x && debouncer.seconds()>0.2) {
            debouncer.reset();
            startPosition = (liftMotor_Left.getCurrentPosition()+liftMotor_Right.getCurrentPosition())/2;
            targetPosition = upTarget;
            pid.setSetPoint(1.0); // Full progress
        }

        if (gamepad1.y && debouncer.seconds()>0.2) {
            startPosition = (liftMotor_Left.getCurrentPosition()+liftMotor_Right.getCurrentPosition())/2;
            targetPosition = downTarget;
            pid.setSetPoint(1.0); // Full progress
        }

        int currentPosition = (liftMotor_Left.getCurrentPosition()+liftMotor_Right.getCurrentPosition())/2;

        double denominator = targetPosition - startPosition;
        if (denominator == 0) denominator = 1;

        double progress = (double) (currentPosition - startPosition) / denominator;
        progress = Math.max(0.0, Math.min(1.0, progress));

        double output = pid.calculate(progress);
        output = Math.max(0, Math.min(1.0, output));

        // If retracting (down target), invert the output signal.
        if (targetPosition < startPosition) {
            output = -output;
        }

        liftMotor_Left.setPower(output);
        liftMotor_Right.setPower(output);

        telemetry.addData("Start Pos", startPosition);
        telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("Current Pos Left", liftMotor_Left.getCurrentPosition());
        telemetry.addData("Current Pos Right", liftMotor_Right.getCurrentPosition());
        telemetry.addData("Progress", progress);
        telemetry.addData("Power", output);
    }
}
