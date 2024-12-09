package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotMovement {
    private DriveTrainControlMode driveTrainControlMode;

    //Declare gamepad
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    //Declare robot
    private final RobotHardware robot;

    //Declare the motor max speed
    private final double motorMaxSpeed;

    //Declare constructor
    public RobotMovement (Gamepad gamepad1, Gamepad gamepad2, RobotHardware robot, double motorMaxSpeed) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;
        this.motorMaxSpeed = motorMaxSpeed;
        this.driveTrainControlMode = DriveTrainControlMode.ROBOT_CENTRIC;
    }

    //Set the debounce timer
    private final ElapsedTime debounceTimer = new ElapsedTime();
    private static final double DEBOUNCE_THRESHOLD = 0.25;

    public void robotDriveTrain() {

        //Set the imu parameters
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double x = 0;
        double y = 0;
        double rx = 0;

        //Set the gamepad parameters
        if (Math.abs(gamepad1.right_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {
            y = -gamepad1.right_stick_y;
            x = gamepad1.right_stick_x;
            rx = gamepad1.left_stick_x;
        } else if (Math.abs(gamepad2.right_stick_y) > 0.1 || Math.abs(gamepad2.right_stick_x) > 0.1 || Math.abs(gamepad2.left_stick_x) > 0.1) {
            y = -gamepad2.right_stick_y;
            x = gamepad2.right_stick_x;
            rx = gamepad2.left_stick_x;
        }

        /**Future note, check if short circuit is needed**/
        if ((gamepad1.start && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) || (gamepad2.start && debounceTimer.seconds() > DEBOUNCE_THRESHOLD)) {
            debounceTimer.reset();
            if (driveTrainControlMode == DriveTrainControlMode.ROBOT_CENTRIC) {
                driveTrainControlMode = DriveTrainControlMode.FIELD_CENTRIC;
            }
            else {
                driveTrainControlMode = DriveTrainControlMode.ROBOT_CENTRIC;
            }
        }

        if ((gamepad1.back && debounceTimer.seconds() > DEBOUNCE_THRESHOLD) || (gamepad2.back && debounceTimer.seconds() > DEBOUNCE_THRESHOLD)) {
            debounceTimer.reset();
            robot.imu.resetYaw();
        }

        if (driveTrainControlMode == DriveTrainControlMode.ROBOT_CENTRIC) {
            //Robot Centric drive power calculations
            double robotCentricDenominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double robotCentricFrontLeftMotorPower = ((y + x + rx) * motorMaxSpeed) / robotCentricDenominator;
            double robotCentricBackLeftMotorPower = ((y - x + rx) * motorMaxSpeed) / robotCentricDenominator;
            double robotCentricFrontRightMotorPower = ((y - x - rx) * motorMaxSpeed) / robotCentricDenominator;
            double robotCentricBackRightMotorPower = ((y + x - rx) * motorMaxSpeed) / robotCentricDenominator;

            //Set the motor power
            robot.frontLeftMotor.setPower(robotCentricFrontLeftMotorPower);
            robot.backLeftMotor.setPower(robotCentricBackLeftMotorPower);
            robot.frontRightMotor.setPower(robotCentricFrontRightMotorPower);
            robot.backRightMotor.setPower(robotCentricBackRightMotorPower);
        }

        else {
            //Field Centric drive power calculations
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double fieldCentricDenominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double fieldCentricFrontLeftMotorPower = ((rotY + rotX + rx) * motorMaxSpeed) / fieldCentricDenominator;
            double fieldCentricBackLeftMotorPower = ((rotY - rotX + rx) * motorMaxSpeed) / fieldCentricDenominator;
            double fieldCentricFrontRightMotorPower = ((rotY - rotX - rx) * motorMaxSpeed) / fieldCentricDenominator;
            double fieldCentricBackRightMotorPower = ((rotY + rotX - rx) * motorMaxSpeed) / fieldCentricDenominator;

            robot.frontLeftMotor.setPower(fieldCentricFrontLeftMotorPower);
            robot.backLeftMotor.setPower(fieldCentricBackLeftMotorPower);
            robot.frontRightMotor.setPower(fieldCentricFrontRightMotorPower);
            robot.backRightMotor.setPower(fieldCentricBackRightMotorPower);
        }
    }

    //Declare the enum
    private enum DriveTrainControlMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }
}
