package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;

import java.util.Locale;

public class PinPoint {
    RobotHardware robot;

    public PinPoint(RobotHardware robot) {
        this.robot = robot;
    }

    public void initPinPoint() {
        robot.pinPoint.setOffsets(-149.225, -165.1); //these are tuned for 3110-0002-0001 Product Insight #1
        robot.pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        robot.pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinPoint.resetPosAndIMU();
    }

    public String getPose() {
        robot.pinPoint.update();
        Pose2D pos = robot.pinPoint.getPosition();

        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
    }
}
