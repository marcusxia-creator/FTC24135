package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SampleLoop extends LinearOpMode {
    TrajectoryFollower follower;
    TrajectoryFollower.TrapezoidalMotionProfile profile;
    Odometry2D Odometry2D;
    MecanumDrive MecanumDrive;


    double totalDistance = 48.0; // inches
    double maxVelocity = 30.0;   // in/sec
    double maxAcceleration = 60.0; // in/sec^2

    double startTime;
    Pose2D startPose = new Pose2D(0, 0, 0);
    Pose2D endPose = new Pose2D(totalDistance, 0, 0);

    @Override
    public void runOpMode() {
        // Example PID and FF values
        follower = new TrajectoryFollower(
                new TrajectoryFollower.PIDCoefficients(0.05, 0, 0.002),
                new TrajectoryFollower.PIDCoefficients(0.05, 0, 0.002),
                new TrajectoryFollower.PIDCoefficients(0.02, 0, 0.001),
                1.0, 0.0, 0.05  // kV, kA, kStatic
        );

        profile = new TrajectoryFollower.TrapezoidalMotionProfile(totalDistance, maxVelocity, maxAcceleration);

        waitForStart();

        while (opModeIsActive()) {

            // Get current pose (from odometry)
            Pose2D currentPose = new Pose2D(Odometry2D.getPoseEstimate().getX(),Odometry2D.getPoseEstimate().getY(),Odometry2D.getPoseEstimate().getHeading());


            // Update target pose based on progress along path
            double progress = Math.min(totalDistance,  currentPose.minus(startPose).norm()); // crude approximation
            Pose2D targetPose = new Pose2D(
                    startPose.getX() + (endPose.getX() - startPose.getX()) * (progress / totalDistance),
                    startPose.getY() + (endPose.getY() - startPose.getY()) * (progress / totalDistance),
                    0
            );

            double velocity = profile.getVelocityAtDistance(progress);
            double acceleration = profile.getAccelerationAtDistance(progress);

            // Project velocity and acceleration
            Pose2D targetVelocity = TrajectoryFollower.projectScalarMotion(
                    startPose, endPose, velocity, acceleration, 0, 0
            );
            Pose2D targetAcceleration = TrajectoryFollower.projectScalarAcceleration(
                    startPose, endPose, acceleration, 0
            );

            // Compute control signal
            Pose2D driveCommand = follower.update(currentPose, targetPose, targetVelocity, targetAcceleration);

            // Send to drivetrain
            MecanumDrive.setDrivePower(new Pose2d(driveCommand.getX(),driveCommand.getY(),driveCommand.getHeading()));


            // Optional: stop condition
            if (progress >= totalDistance && driveCommand.norm() < 0.05) {
                MecanumDrive.setDrivePower(new Pose2d(0, 0, 0));
                break;
            }


            telemetry.addData("TargetX", targetPose.getX());
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Drive VX", driveCommand.getX());
            telemetry.addData("Drive VY", driveCommand.getY());
            telemetry.addData("Drive Omega", driveCommand.getHeading());
            telemetry.update();

        }

    }

}
