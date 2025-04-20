package org.firstinspires.ftc.teamcode.Motion;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Odometry2D {
    private GoBildaPinpointDriver odometryModule; // Your GoBilda Odometry Computer driver

    public static double x_offset = -149.225; // unit in mm
    public static double y_offset = -165.1; // unit in mm

    public Odometry2D(GoBildaPinpointDriver module) {
        this.odometryModule = module;
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        odometryModule.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //
        this.odometryModule.setOffsets(x_offset,y_offset);
        this.odometryModule.resetPosAndIMU();
        //txWorldPinpoint = initialPose;
    }

    /**
     * Call this in your loop to refresh odometry data.
     * Only needed if your driver requires a manual update.
     */
    public void update() {
        odometryModule.update();
    }

    /**
     * Returns the current estimated pose from the odometry computer (field-centric).
     */
    public Pose2d getPoseEstimate() {
        return new Pose2d(
                odometryModule.getPosition().getX(DistanceUnit.INCH),
                odometryModule.getPosition().getY(DistanceUnit.INCH),
                Math.toRadians(odometryModule.getPosition().getHeading(AngleUnit.DEGREES))
        );
    }

    /**
     * Returns the current velocity from the odometry computer (field-centric).
     */
    public Pose2d getPoseVelocity() {
        return new Pose2d(
                odometryModule.getVelX(), // mm/sec
                odometryModule.getVelY(),
                Math.toRadians(odometryModule.getHeadingVelocity())
        );
    }

    /**
     * Returns the robot-relative velocity (for control loops).
     */
    public Pose2d getRobotRelativeVelocity() {
        double vx = odometryModule.getVelX(); // mm/sec
        double vy = odometryModule.getVelY();
        double omega = Math.toRadians(odometryModule.getHeadingVelocity());

        double heading = odometryModule.getPosition().getHeading(AngleUnit.RADIANS);

        double vxRobot =  vx * Math.cos(heading) + vy * Math.sin(heading);
        double vyRobot = -vx * Math.sin(heading) + vy * Math.cos(heading);

        return new Pose2d(vxRobot, vyRobot, omega);
    }

    /**
     * Resets the odometry computer's internal X, Y, and heading to zero.
     */
    public void reset() {
        odometryModule.resetPosAndIMU();
    }
}

