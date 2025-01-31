package org.firstinspires.ftc.teamcode.Auto.Roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Roadrunner.drive.GoBildaPinpointDriver;
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

    private GoBildaPinpointDriver pinpoint;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"Pinpoint");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint.resetPosAndIMU();

        waitForStart();
        while (!isStopRequested()) {
            pinpoint.update();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("X Encoder Value: ", pinpoint.getEncoderX());
            telemetry.addData("Y Encoder Value: ", pinpoint.getEncoderY());
            telemetry.addData("Heading Value: ", pinpoint.getHeading());
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Estimated Position",pinpoint.getPosition());
            telemetry.update();
        }
    }
}
