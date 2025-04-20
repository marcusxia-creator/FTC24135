package org.firstinspires.ftc.teamcode.TeleOps;

import static com.qualcomm.robotcore.util.Range.clip;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp (name = "PIDFArm")
public class PIDFArm extends OpMode {
    private PIDController controller;

    public static  double p = 0, i = 0 , d = 0; //p1=0.0009,d1=0.00001,f=0.01
    public  static double f=0;

    public static int targetPosition =0;
    public static int startPosition = 0;
    public static  double speed = 0;

    private final double ticks_in_degree = 537.7/360;

    private DcMotor deposit_Slide_Left;
    private DcMotorEx deposit_Slide_Right;
    /**
     private ColorSensor colorSensor;

     private int redValue;
     private int greenValue;
     private int blueValue;

     **/
    @Override
    public void init() {
        controller = new PIDController(p, i, d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deposit_Slide_Left = hardwareMap.get(DcMotor.class, "VS_Motor_Right");
        //deposit_Slide_Right = hardwareMap.get(DcMotorEx.class, "VS_Motor_Right");

        deposit_Slide_Left.setDirection(DcMotor.Direction.FORWARD);
        deposit_Slide_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deposit_Slide_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startPosition = deposit_Slide_Left.getCurrentPosition();
    }

    @Override
    public void loop(){
        controller.setPID(p,i,d);
        int currentPosition= deposit_Slide_Left.getCurrentPosition();
        //int armPos_right = deposit_Slide_Right.getCurrentPosition();
        double denominator = targetPosition - startPosition;
        if (denominator == 0) denominator = 1; // quick protection

        double progress = (double) (currentPosition - startPosition )/denominator;

        // Clip to avoid over/underflow
        progress = Math.max(0.0, Math.min(1.0, progress));
        
        double pid_left = controller.calculate(progress);

        //double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double ff = 1 * f;

        double power_left = pid_left + ff;

        power_left = Math.max(-1.0, Math.min(1.0, power_left));

        deposit_Slide_Left.setPower(power_left);
        /**
        deposit_Slide_Left.setTargetPosition(targetPosition);
        deposit_Slide_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        deposit_Slide_Left.setPower(speed);
        */
         //deposit_Slide_Right.setTargetPosition(target);
        //deposit_Slide_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //deposit_Slide_Right.setPower(speed);

        telemetry.addData("current position ", currentPosition);
        telemetry.addData("pos_left power", power_left);
        telemetry.addData("pid value ", pid_left);
        telemetry.addData("progress ", progress);
        telemetry.addData("target int ", targetPosition);
        //telemetry.addData("Color Sensor ", colorSensor.red());

        telemetry.update();
    }
}

