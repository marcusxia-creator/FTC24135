package org.firstinspires.ftc.teamcode.Motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumDrive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;

        // Reverse the right side motors if needed
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /**
     * Applies normalized drive power to the mecanum drive.
     * The input Pose2d contains robot-centric velocity: x = forward, y = strafe, heading = turn.
     */
    public void setDrivePower(Pose2d drivePower) {
        double x = drivePower.getX();       // Forward/backward
        double y = drivePower.getY();       // Left/right strafe
        double turn = drivePower.getHeading(); // Rotational velocity

        // Basic mecanum kinematics
        double fl = x + y + turn;
        double fr = x - y - turn;
        double bl = x - y + turn;
        double br = x + y - turn;

        // Normalize powers to fit within [-1, 1]
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
