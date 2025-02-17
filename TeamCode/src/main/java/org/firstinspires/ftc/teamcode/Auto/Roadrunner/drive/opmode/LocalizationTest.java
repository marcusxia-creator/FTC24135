package org.firstinspires.ftc.teamcode.Auto.Roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Roadrunner.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y *0.5,
                            -gamepad1.left_stick_x *0.5,
                            -gamepad1.right_stick_x *0.5
                    )
            );

            if (gamepad1.a){
                drive.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            Pose2d poseVelocity = drive.getPoseVelocity();
            telemetry.addData("x poseEstimate inch", poseEstimate.getX());
            telemetry.addData("y poseEstimate inch", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("velocity pose getX", poseVelocity.getX());
            telemetry.addData("velocity pose gety", poseVelocity.getY());
            telemetry.addData("velocity from getPoseVelocity -x ", drive.getPoseVelocity().getX());
            telemetry.addData("velocity from getPoseVelocity -y ", drive.getPoseVelocity().getY());
            telemetry.update();
        }
    }
}
