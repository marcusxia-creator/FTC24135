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

    //Hardware Definition
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    public GoBildaPinpointDriver odo;

    //global odo data
    public Pose2D currentPos;
    public Pose2D currentVel;

    //Power Pose2D object
    public Pose2D targetPower;

    //Velocity Pose2D object
    public Pose2D targetVel;

    //Velocity to Power PID Controllers
    public PIDController vController;
    public PIDController vRotController;

    //Position Pose2D object
    public Pose2D startingPos;
    public Pose2D targetPos;

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



    public void InitDrive(CONTROLMODE InitMode, Pose2D initPose, boolean fieldCentric){
        controlMode = InitMode;
        this.fieldCentric = fieldCentric;

        frontLeftMotor  = robot.frontLeftMotor;
        backLeftMotor   = robot.backLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor  = robot.backRightMotor;

        vController = IceWaddlerConfig.vController;
        vRotController = IceWaddlerConfig.vRotController;

        InitOdo(initPose);
    }

    public void InitOdo(Pose2D initPose){
        odo=robot.odo;

        odo.setOffsets(IceWaddlerConfig.odoXOffset, IceWaddlerConfig.odoYOffset); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(IceWaddlerConfig.odoEncoderResolution);
        odo.setEncoderDirections(IceWaddlerConfig.xEncoderDirection, IceWaddlerConfig.yEncoderDirection);
        //Set to start counting at initPose parameter
        odo.resetPosAndIMU();
        odo.setPosition(initPose);

        updateOdo();
    }

    public void updateOdo(){
        odo.update();

        currentPos = odo.getPosition();
        currentVel = odo.getVelocity();
    }

    public void setFieldCentric(boolean v){
        fieldCentric = v;
    }

    public void toggleFieldCentric(){
        fieldCentric = !fieldCentric;
    }

    public void runByPower(Pose2D targetPower){
        controlMode = CONTROLMODE.POWER;
        this.targetPower = targetPower;
    }

    private void writePower(){

        Pose2D robotCentricPower = rotatePose(targetPower, AngleUnit.RADIANS, -currentPos.getHeading(AngleUnit.RADIANS));

        double x = robotCentricPower.getX(DistanceUnit.METER);
        double y = robotCentricPower.getY(DistanceUnit.METER);
        double rot = robotCentricPower.getHeading(AngleUnit.RADIANS);

        frontLeftMotor.setPower(-x-y+rot);
        backLeftMotor.setPower(-x+y+rot);
        frontRightMotor.setPower(x-y+rot);
        backRightMotor.setPower(x+y+rot);
    }

    public void runByVel(Pose2D targetVel){
        controlMode = CONTROLMODE.VELOCITY;
        this.targetVel = targetVel;
    }

    private void writeVel() {

        targetPower = new Pose2D(
                DistanceUnit.METER,
                vController.calculate(currentVel.getX(DistanceUnit.METER), targetVel.getX(DistanceUnit.METER)),
                vController.calculate(currentVel.getY(DistanceUnit.METER), targetVel.getY(DistanceUnit.METER)),
                AngleUnit.RADIANS,
                vRotController.calculate(currentVel.getHeading(AngleUnit.RADIANS), targetVel.getHeading(AngleUnit.RADIANS)));

        writePower();
    }

    public void loop(){

        updateOdo();

        switch (controlMode){
            case POWER:
                //Apply robot centric with fresh odo data
                if(!fieldCentric){
                    targetPower=rotatePose(targetPower, AngleUnit.RADIANS, currentPos.getHeading(AngleUnit.RADIANS));
                }
                writePower();
                break;

            case VELOCITY:
                //Apply robot centric with fresh odo data
                if(!fieldCentric){
                    targetVel=rotatePose(targetVel, AngleUnit.RADIANS, currentPos.getHeading(AngleUnit.RADIANS));
                }
                writeVel();
                break;

            case PTP:
                //Not yet implemented
                break;
        }
    }

    //Helper Function
    private Pose2D rotatePose(Pose2D pose, AngleUnit angleUnit, double angle){

        angle=AngleUnit.RADIANS.fromUnit(angleUnit,angle);

        return new Pose2D(
                DistanceUnit.METER,
                pose.getX(DistanceUnit.METER)*Math.cos(angle)-pose.getY(DistanceUnit.METER)*Math.sin(angle),
                pose.getX(DistanceUnit.METER)*Math.sin(angle)+pose.getY(DistanceUnit.METER)*Math.cos(angle),
                AngleUnit.RADIANS,
                pose.getHeading(AngleUnit.RADIANS)
        );
    }
}