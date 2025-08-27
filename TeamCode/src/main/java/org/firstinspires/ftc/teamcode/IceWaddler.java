package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

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


    //Position to Velcity PID Controllers
    public PIDController pLatController;
    public PIDController pRotController;

    //Position Pose2D object
    public Pose2D startingPos;
    public Pose2D targetPos;
    public boolean decelerate;

    //publicly referencable variables for actions
    public double completion;
    public double distanceTraveled;
    public double distanceRemaining;

    public enum CONTROLMODE {
        POWER,
        VELOCITY,
        POSITION,
        PATH
    }

    public boolean fieldCentric=false;

    public CONTROLMODE controlMode;

    public IceWaddler(RobotHardware robot){
        this.robot=robot;
    }

    public void Init(CONTROLMODE InitMode, Pose2D initPose, boolean fieldCentric){
        controlMode = InitMode;
        this.fieldCentric = fieldCentric;

        frontLeftMotor  = robot.frontLeftMotor;
        backLeftMotor   = robot.backLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor  = robot.backRightMotor;

        vController = fromCoeffs(IceWaddlerConfig.vController);
        vRotController = fromCoeffs(IceWaddlerConfig.vRotController);

        pLatController = fromCoeffs(IceWaddlerConfig.pLatController);
        pRotController = fromCoeffs(IceWaddlerConfig.pRotController);

        InitOdo(initPose);
    }

    public void InitOdo(Pose2D initPose){
        odo = robot.odo;

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

        Pose2D robotCentricPower = rotatePose(targetPower, AngleUnit.RADIANS, currentPos.getHeading(AngleUnit.RADIANS));

        double x = robotCentricPower.getX(DistanceUnit.METER);
        double y = robotCentricPower.getY(DistanceUnit.METER);
        double rot = robotCentricPower.getHeading(AngleUnit.RADIANS);

        //Write to Mecanum drive

        frontLeftMotor.setPower(x+y+rot);
        backLeftMotor.setPower(x-y+rot);
        frontRightMotor.setPower(x-y-rot);
        backRightMotor.setPower(x+y-rot);
    }

    public void runByVel(Pose2D targetVel){
        controlMode = CONTROLMODE.VELOCITY;
        this.targetVel = targetVel;
    }

    private void writeVel() {

        targetPower = new Pose2D(
                DistanceUnit.METER,
                vController.calculate(currentVel.getX(DistanceUnit.METER), targetVel.getX(DistanceUnit.METER)),
                vController.calculate(-currentVel.getY(DistanceUnit.METER), targetVel.getY(DistanceUnit.METER)),
                AngleUnit.RADIANS,
                -vRotController.calculate(currentVel.getHeading(AngleUnit.RADIANS), targetVel.getHeading(AngleUnit.RADIANS)));

        writePower();
    }

    public void runByPos(Pose2D startingPos, Pose2D targetPos, boolean decelerate){
        controlMode = CONTROLMODE.POSITION;
        this.startingPos=startingPos;
        this.targetPos=targetPos;
        this.decelerate=decelerate;
    }

    public void writePos(){
        //Line Constants
        double A = startingPos.getY(DistanceUnit.METER)-targetPos.getY(DistanceUnit.METER);
        double B = targetPos.getX(DistanceUnit.METER)-startingPos.getX(DistanceUnit.METER);
        double C = startingPos.getX(DistanceUnit.METER)*targetPos.getY(DistanceUnit.METER)-targetPos.getX(DistanceUnit.METER)*startingPos.getY(DistanceUnit.METER);
        //Target line can be graphed as Ax+By+C=0

        //Lateral PID correction
        double latDistance = (A*currentPos.getX(DistanceUnit.METER)+B*currentPos.getY(DistanceUnit.METER)+C)/
                Math.sqrt(Math.pow(A,2)+Math.pow(B,2)); //Add Desmos link
        double latCorrection = pLatController.calculate(latDistance);

        distanceTraveled = Math.sqrt(Math.pow(distanceBetween(startingPos, currentPos, DistanceUnit.METER),2)-Math.pow(latDistance,2));
        distanceRemaining = Math.sqrt(Math.pow(distanceBetween(currentPos, targetPos, DistanceUnit.METER),2)-Math.pow(latDistance,2));
        double totalDistance = distanceTraveled+distanceRemaining;
        completion = distanceTraveled/totalDistance;

        //Decel
        double lonCorrection = Range.clip(Math.sqrt(Math.pow(IceWaddlerConfig.minSpeed,2)+2*IceWaddlerConfig.maxDecel*distanceRemaining),
                IceWaddlerConfig.minSpeed,IceWaddlerConfig.maxSpeed); //Add Desmos link
        //PID will handle acceleration

        //Rotation control, changes linearly over distance
        double modOffset = 0; //To minimize required movement, see Desmos
        if(startingPos.getHeading(AngleUnit.DEGREES)-targetPos.getHeading(AngleUnit.DEGREES)<-180){
            modOffset = 2*Math.PI;
        }
        else if(startingPos.getHeading(AngleUnit.DEGREES)-targetPos.getHeading(AngleUnit.DEGREES)>180){
            modOffset = -2*Math.PI;
        }

        double rotSetpoint = startingPos.getHeading(AngleUnit.RADIANS)+completion*(targetPos.getHeading(AngleUnit.RADIANS)-startingPos.getHeading(AngleUnit.RADIANS)+modOffset);

        double rotCorrection = pRotController.calculate(((rotSetpoint-currentPos.getHeading(AngleUnit.RADIANS)+Math.PI)%(2*Math.PI))-Math.PI);

        Pose2D OrientedVel = new Pose2D(DistanceUnit.METER, latCorrection, lonCorrection, AngleUnit.RADIANS, rotCorrection);
        
        //Align movement to line
        targetVel = rotatePose(OrientedVel, AngleUnit.RADIANS, Math.atan((startingPos.getY(DistanceUnit.METER)-targetPos.getY(DistanceUnit.METER))/(startingPos.getX(DistanceUnit.METER)-targetPos.getX(DistanceUnit.METER))));
        
        writeVel();
    }

    public void loop(){

        updateOdo();

        switch (controlMode){
            case POWER:
                //Apply robot centric with fresh odo data
                if(!fieldCentric){
                    targetPower=rotatePose(targetPower, AngleUnit.RADIANS, -currentPos.getHeading(AngleUnit.RADIANS));
                }
                writePower();
                break;

            case VELOCITY:
                //Apply robot centric with fresh odo data
                if(!fieldCentric){
                    targetVel=rotatePose(targetVel, AngleUnit.RADIANS, -currentPos.getHeading(AngleUnit.RADIANS));
                }
                writeVel();
                break;

            case POSITION:
                //Not yet implemented
                break;
        }
    }

    //Helper Functions
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

    private double distanceBetween(Pose2D pose1, Pose2D pose2, DistanceUnit distanceUnit){
        return Math.sqrt(
                Math.pow(pose1.getX(distanceUnit)-pose2.getX(distanceUnit),2)+
                        Math.pow(pose1.getY(distanceUnit)-pose2.getY(distanceUnit),2)
        );
    }

    private PIDController fromCoeffs(PIDCoefficients Coeffs){
        return new PIDController(Coeffs.p, Coeffs.i, Coeffs.d);
    }
}