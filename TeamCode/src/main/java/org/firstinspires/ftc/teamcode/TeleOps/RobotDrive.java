package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddlerAction;

import java.util.Arrays;

/** Button Config for Drive
 * * Joy Right Y                : Drive
 * * Joy Right X                : Strafe
 * * Joy Left X                 : Turn
 * * Left Trigger               : Fine Movement + Joystick
 * * START                      : Field centric / Robot centric toggle
 * * Right Trigger + Back       : Reset Odometry Counting, for semi auto
 * Gamepad 1 override Gamepad 2
 */

public class RobotDrive {

    private final GamepadEx gamepad_1;
    private final GamepadEx gamepad_2;
    private final RobotHardware robot;

    public enum DRIVESTATE{
        MANUAL,

        TO_LOADING,
        TO_SHOOTING
    };
    DRIVESTATE drivestate;

    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing

    private boolean startPressed = false;
    private boolean backPressed = false;

    private double velFactor=RobotActionConfig.velFactor;       //m/s
    private double rotFactor=RobotActionConfig.rotFactor;       //rad/s

    public IceWaddler iceWaddler;

    public RobotDrive(RobotHardware robot, GamepadEx gamepad_1, GamepadEx gamepad_2) {
        this.robot = robot;
        this.gamepad_1 = gamepad_1;
        this.gamepad_2 = gamepad_2;
    }

    public void Init() {
        iceWaddler = new IceWaddler(robot);
        iceWaddler.Init(IceWaddler.CONTROLMODE.VELOCITY,
                new Pose2D(DistanceUnit.METER,0,0,AngleUnit.RADIANS,0),
                true);
        drivestate=DRIVESTATE.MANUAL;
    }


    @SuppressLint("DefaultLocale")
    public void DriveLoop() {
        // Toggle control mode
        if (((gamepad_1.getButton(START)) || gamepad_2.getButton(START)) && !startPressed && (!gamepad_1.getButton(LEFT_BUMPER) || !gamepad_2.getButton(LEFT_BUMPER))) {
            iceWaddler.toggleFieldCentric();
            debounceTimer.reset();
            startPressed = true;
        } else if (!gamepad_1.getButton(START) || !gamepad_2.getButton(START)) {
            startPressed = false;
        }

        //Reset Odo to (0,0)
        if (((gamepad_1.getButton(BACK) && gamepad_1.getButton(RIGHT_BUMPER)) || (gamepad_2.getButton(BACK) && gamepad_2.getButton(RIGHT_BUMPER))) && !backPressed) {
            iceWaddler.InitOdo(new Pose2D(DistanceUnit.METER,0, 0, AngleUnit.DEGREES, 0));
            debounceTimer.reset();
            backPressed = true;
        } else if (!gamepad_1.getButton(BACK) || !gamepad_2.getButton(BACK)) {
            backPressed = false;
        }

        /** Reset IMU heading using button back and reset odometry
         * * need to remove this feature, as back button reseting imu may interfer with Roadrunner pose estimation.
         * * back button is using for retracting slide after auto phase and initial start of teleops

        if (gamepad_1.getButton(BACK) || gamepad_2.getButton(BACK) && !backPressed) {
            //robot.initIMU();
            //robot.resetDriveEncoders();
            debounceTimer.reset();
            backPressed = true;
        } else if (!gamepad_1.getButton(BACK) || !gamepad_2.getButton(BACK)) {
            backPressed = false;
        }
         */

        if(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.4 || gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.4){
            double factor = Math.max(gamepad_1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            velFactor = Range.clip(RobotActionConfig.velFactor *(1.4 - factor),0,1); //1.0 - power reduction will be 0.6 - 0.2
        }
        else {
            velFactor = RobotActionConfig.velFactor;
        }
        double drive = 0.0;
        double strafe = 0.0;
        double rotate = 0.0;

        // gamepad 1 take priority override gamepad 2
        if (Math.abs(gamepad_1.getRightY()) > 0.1 || Math.abs(gamepad_1.getRightX()) > 0.1 || Math.abs(gamepad_1.getLeftX()) > 0.1) {
            drive = deadband(-gamepad_1.getRightY(),0.1) * velFactor;
            strafe = deadband(gamepad_1.getRightX(),0.1) * velFactor;
            rotate = -deadband(gamepad_1.getLeftX(),0.1) * rotFactor;
        } else if (Math.abs(gamepad_2.getRightY()) > 0.1 || Math.abs(gamepad_2.getRightX()) > 0.1 || Math.abs(gamepad_2.getLeftX()) > 0.1) {
            drive = deadband(-gamepad_2.getRightY(),0.1) * velFactor;
            strafe = deadband(gamepad_2.getRightX(),0.1) * velFactor;
            rotate = -deadband(gamepad_2.getLeftX(),0.1) * rotFactor;
        }

        //SemiAuto FSM
        switch(drivestate){
            case MANUAL:
                iceWaddler.runByVel(new Pose2D(
                        DistanceUnit.METER,
                        drive,
                        strafe,
                        AngleUnit.RADIANS,
                        rotate),
                        rotate);

                if (((gamepad_1.getButton(A) && gamepad_1.getButton(RIGHT_BUMPER)) || (gamepad_2.getButton(A) && gamepad_2.getButton(RIGHT_BUMPER)))){
                    iceWaddler.runPath(Arrays.asList(new IceWaddlerAction(iceWaddler.currentPos,RobotActionConfig.loadingZone,true)));
                    drivestate = DRIVESTATE.TO_LOADING;
                }

                if (((gamepad_1.getButton(B) && gamepad_1.getButton(RIGHT_BUMPER)) || (gamepad_2.getButton(B) && gamepad_2.getButton(RIGHT_BUMPER)))){
                    iceWaddler.runPath(Arrays.asList(new IceWaddlerAction(iceWaddler.currentPos,RobotActionConfig.launchingPose,true)));
                    drivestate = DRIVESTATE.TO_SHOOTING;
                }

            case TO_LOADING:
                if (iceWaddler.controlMode == IceWaddler.CONTROLMODE.STBY || gamepad_1.getButton(BACK) || gamepad_2.getButton(BACK) || drive!=0 || strafe !=0 || rotate!=0){
                    drivestate = DRIVESTATE.MANUAL;
                }

            case TO_SHOOTING:
                if (iceWaddler.controlMode == IceWaddler.CONTROLMODE.STBY || gamepad_1.getButton(BACK) || gamepad_2.getButton(BACK) || drive!=0 || strafe !=0 || rotate!=0){
                    drivestate = DRIVESTATE.MANUAL;
                }
        }

        //Update Movements
        iceWaddler.loop();

        // Update telemetry with the latest data
        // empty
    }// end of driveloop


    double deadband(double input, double threshold) {
        if (Math.abs(input) < threshold) { // Ignore small values
            return 0.0;
        }
        return input;
    }
}
