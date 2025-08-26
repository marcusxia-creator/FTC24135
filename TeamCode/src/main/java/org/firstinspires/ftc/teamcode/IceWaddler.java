package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class IceWaddler {
    final RobotHardware robot;
    public GoBildaPinpointDriver odo;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    //Velocity to Power PID Controllers
    public PIDController vController;
    public PIDController vRotController;

    public enum CONTROLMODE {
        POWER,
        VELOCITY,
        PTP
    }

    public boolean fieldCentric=false;

    public CONTROLMODE controlMode;

    public IceWaddler(RobotHardware robot){
        this.robot=robot;
    }

    public void InitOdo(Pose2D initPose){
        odo=robot.odo;

        odo.setOffsets(IceWaddlerConfig.odoXOffset, IceWaddlerConfig.odoYOffset); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(IceWaddlerConfig.odoEncoderResolution);
        odo.setEncoderDirections(IceWaddlerConfig.xEncoderDirection, IceWaddlerConfig.yEncoderDirection);
        //Set to start counting at initPose parameter
        odo.resetPosAndIMU();
        odo.setPosition(initPose);
    }

    public void InitDrive(CONTROLMODE InitMode, boolean fieldCentric){
        controlMode = InitMode;
        this.fieldCentric = fieldCentric;

        frontLeftMotor  = robot.frontLeftMotor;
        backLeftMotor   = robot.backLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor  = robot.backRightMotor;

        vController = IceWaddlerConfig.vController;
        vRotController = IceWaddlerConfig.vRotController;
    }

    private void writePower(Pose2D targetPower, Pose2D currentPos, Pose2D currentVel){
        if(fieldCentric) {
            double currentHeading = currentPos.getHeading(AngleUnit.RADIANS);

            targetPower = new Pose2D(DistanceUnit.METER,
                    targetPower.getX(DistanceUnit.METER)*Math.cos(currentHeading)-targetPower.getY(DistanceUnit.METER)*Math.sin(currentHeading),
                    targetPower.getX(DistanceUnit.METER)*Math.sin(currentHeading)+targetPower.getY(DistanceUnit.METER)*Math.cos(currentHeading),
                    AngleUnit.RADIANS, targetPower.getHeading(AngleUnit.RADIANS)
            );
        }

        double x = targetPower.getX(DistanceUnit.METER);
        double y = targetPower.getY(DistanceUnit.METER);
        double rot = targetPower.getHeading(AngleUnit.DEGREES);

        frontLeftMotor.setPower(-x-y+rot);
        backLeftMotor.setPower(-x+y+rot);
        frontRightMotor.setPower(x-y+rot);
        backRightMotor.setPower(x+y+rot);
    }

    private void writeVel(Pose2D targetVel, Pose2D currentPos, Pose2D currentVel) {

        Pose2D targetPower = new Pose2D(
                DistanceUnit.METER,
                vController.calculate(currentVel.getX(DistanceUnit.METER), targetVel.getX(DistanceUnit.METER)),
                vController.calculate(currentVel.getY(DistanceUnit.METER), targetVel.getY(DistanceUnit.METER)),

                AngleUnit.RADIANS,
                vRotController.calculate(currentVel.getHeading(AngleUnit.RADIANS), targetVel.getHeading(AngleUnit.RADIANS)));

        writePower(targetPower, currentPos, currentVel);
    }

}
