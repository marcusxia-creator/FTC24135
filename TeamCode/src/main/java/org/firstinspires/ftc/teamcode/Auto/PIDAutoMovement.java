package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.GoBildaPinpointDriver;

public class PIDAutoMovement {
    public double integral;
    public double lastErrorHeading;
    public double lastErrorDrive;

    public static double hP;
    public static double hI;
    public static double hD;

    public static double dP;
    public static double dD;

    public static double sP;
    public static double sD;

    GoBildaPinpointDriver odo;
    RobotHardware robot;

    // initial pose coordinate
    /*
        /\ Y direction toward blue Alliance.
        |
        |
        |
        |
        ____________________ \   X direction parallel on Red Alliance and toward audience
                             /
         Angle facing to audience is 0 deg.
         */

    public void setPose(Pose2D pose){
        odo.setPosition(pose); // pose based on FTC field coordinate
    }

    public double pidHeading( double target, double kp, double ki, double kd, double current) {
        double error  = target - current;
        integral += error;
        double derivative = error - lastErrorHeading;
        if (error > Math.PI){
            error -= Math.PI*2;
        } else if (error< -Math.PI){
            error +=Math.PI*2;
        }
        double correction = (error*kp) + (integral*ki) +(derivative *kd);
        lastErrorHeading = error;
        return correction;
    }

    public double pfdDrive(double kp, double kd, double kf, double error){
        double derivative = error -lastErrorDrive;
        double correction = (error*kp) + (derivative*kd);
        correction += Math.signum(error)*kf;
        return correction;
    }

    public void goToPosition(double targetX, double targetY, double targetH, double speed){
        Vector2d driveVector = new Vector2d( targetX-odo.getPosition().getX(DistanceUnit.INCH),targetY - odo.getPosition().getY(DistanceUnit.INCH));
        Vector2d rotatedVector = driveVector.rotated(odo.getHeading());

        double inputTurn = pidHeading(targetH, hP, hI, hD, odo.getHeading());
        double driveCorrection = pfdDrive(dP,dD,0, rotatedVector.getX());
        double strafeCorrection = pfdDrive(sP,sD,0, rotatedVector.getY());

        robot.frontLeftMotor.setPower((driveCorrection + strafeCorrection + inputTurn)*speed);
        robot.frontRightMotor.setPower((driveCorrection - strafeCorrection - inputTurn)*speed);
        robot.backRightMotor.setPower((driveCorrection + strafeCorrection - inputTurn)*speed);
        robot.backLeftMotor.setPower((driveCorrection - strafeCorrection + inputTurn)*speed);
    }
}
