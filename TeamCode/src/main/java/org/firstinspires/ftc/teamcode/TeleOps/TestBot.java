package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp (name = "limitswitch Testbot", group = "org.firstinspires.ftc.teamcode")
public class TestBot extends OpMode {

    public DcMotorEx motor1;
    public DigitalChannel limitswitch;
    public HuskyLens webcam;

    double DEBOUNCE_THRESHOLD = 0.2;
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    RUNSTATE runState = RUNSTATE.FALSE;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "Motor1");
        limitswitch = hardwareMap.get(DigitalChannel.class,"LimitSwitch1");
        webcam = hardwareMap.get(HuskyLens.class, "HuskyLens");

        telemetry.addData("initialized","Robot Initialized");
        telemetry.update();
        motor1.setPower(0);
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        if (gamepad1.a && isButtonDebounced()){
            runState=RUNSTATE.TRUE;
        }
        if ((gamepad1.b && isButtonDebounced()) || isSwitchPressed()){
            runState = RUNSTATE.FALSE;
        }
        run();
        telemetry.addData("Limitswitch state", limitswitch.getState());
        telemetry.addData("motor velocity", motor1.getVelocity());
        telemetry.addData("motor power", motor1.getPower());
        telemetry.update();
    }

    private boolean isSwitchPressed(){
        return limitswitch.getState();
    }


    // Debouncer helper
    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
    //Deposit Claw Switch Handler
    private void run() {
        if (runState == RUNSTATE.TRUE) {
            motor1.setPower(0.5);
        } else {
            motor1.setPower(0);
        }
    }

    //Toggle Deposit Claw helper -- Open - Close
    public enum RUNSTATE{
        TRUE,
        FALSE
    }

}
