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

    public PIDController vController;
    public PIDController rotVController;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

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
        vController = IceWaddlerConfig.vController;
        rotVController = IceWaddlerConfig.rotVController;

        controlMode = InitMode;
        this.fieldCentric = fieldCentric;

        frontLeftMotor  = robot.frontLeftMotor;
        backLeftMotor   = robot.backLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor  = robot.backRightMotor;
    }

    void runPower(Pose2D targetPower, Pose2D currentPose){
        if(fieldCentric) {
            targetPower = new Pose2D(DistanceUnit.METER,
                    targetPower.getX(DistanceUnit.METER)*Math.cos(currentPose.getHeading(AngleUnit.RADIANS))-currentPose.getY(DistanceUnit.METER)*Math.sin(targetPower.getHeading(AngleUnit.RADIANS)),
                    targetPower.getX(DistanceUnit.METER)*Math.sin(currentPose.getHeading(AngleUnit.RADIANS))+currentPose.getY(DistanceUnit.METER)*Math.cos(targetPower.getHeading(AngleUnit.RADIANS)),
                    AngleUnit.RADIANS, targetPower.getHeading(AngleUnit.RADIANS)
            );
        }

        double x = targetPower.getX(DistanceUnit.METER);
        double y = targetPower.getY(DistanceUnit.METER);
        double rot = targetPower.getHeading(AngleUnit.DEGREES;

        frontLeftMotor.setPower(-x-y+rot);
        backLeftMotor.setPower(-x+y+rot);
        frontRightMotor.setPower(x-y+rot);
        backRightMotor.setPower(x+y+rot);
    }
}
