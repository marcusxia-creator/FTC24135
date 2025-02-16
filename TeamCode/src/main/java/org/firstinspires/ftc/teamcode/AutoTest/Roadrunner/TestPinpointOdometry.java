package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.drive.SampleMecanumDrive;

@TeleOp(name = "Test_Pinpoint_Odometry", group = "TeleOps")
public class TestPinpointOdometry extends LinearOpMode {

    @Override
    public void runOpMode() {
        // ✅ Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Ready! Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ✅ Set an initial pose (if needed)
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        while (opModeIsActive()) {
            // ✅ Update drive and odometry

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.right_stick_y*0.5,
                            -gamepad1.right_stick_x*0.5,
                            -gamepad1.left_stick_x*0.5
                    )
            );

            drive.update();

            if (gamepad1.a){
                drive.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            // ✅ Retrieve the current position
            Pose2d pose = drive.getPoseEstimate();
            Pose2d velocity = drive.getPoseVelocity();

            // ✅ Display the position and velocity in telemetry
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Velocity X", velocity.getX());
            telemetry.addData("Velocity Y", velocity.getY());
            telemetry.addData("Velocity Heading (deg/sec)", Math.toDegrees(velocity.getHeading()));
            telemetry.update();
        }
    }
}

