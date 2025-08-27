package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddler;
import org.firstinspires.ftc.teamcode.IceWaddlerAction;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "IWPathTester", group = "org.firstinspires.ftc.teamcode.Auto")

public class IceWaddlerPathTest1 extends LinearOpMode {

    RobotHardware robot;

    IceWaddler iceWaddler;

    double lastTickTimestamp = getRuntime();

    double lastPoseLog=getRuntime();

    FtcDashboard dashboard;

    List<Pose2D> Pose2DLog = new ArrayList<>();

    IceWaddlerAction startDelay = new IceWaddlerAction(3,true);
    IceWaddlerAction Movement1 = new IceWaddlerAction(new Pose2D(DistanceUnit.METER,0,0, AngleUnit.RADIANS,0), new Pose2D(DistanceUnit.METER,1,0.5, AngleUnit.DEGREES,90),true);

    List<IceWaddlerAction> Path = Arrays.asList(startDelay,Movement1);

    @Override
    public void runOpMode() {
        //Init IW, with initial pos as needed
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initPinPoint();

        iceWaddler = new IceWaddler(robot);
        iceWaddler.Init(IceWaddler.CONTROLMODE.VELOCITY, new Pose2D(DistanceUnit.METER,0,0,AngleUnit.DEGREES,0), false);

        //Try different poses for testing
        iceWaddler.runPath(Path);

        //Enable dashboard telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        resetRuntime();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ✅ Update drive and odometry
            iceWaddler.loop();

            telemetry.addData("Status", "Initialized");
            telemetry.addData("IceWaddler Status", iceWaddler.controlMode.toString());
            telemetry.addData("IceWaddler Action Index", iceWaddler.currentActionIndex);
            telemetry.addData("X offset", iceWaddler.odo.getXOffset());
            telemetry.addData("Y offset", iceWaddler.odo.getYOffset());
            telemetry.addData("Device Version Number:", iceWaddler.odo.getDeviceVersion());
            telemetry.addData("Device Scalar", iceWaddler.odo.getYawScalar());

            telemetry.addLine("---------PinPoint X, Y---------");
            String data = String.format(Locale.US, "{X inch: %.3f, Y inch: %.3f, H Radian %.3f}", iceWaddler.currentPos.getX(DistanceUnit.INCH), iceWaddler.currentPos.getY(DistanceUnit.INCH), iceWaddler.currentPos.getHeading(AngleUnit.RADIANS));
            String data2 = String.format(Locale.US, "{X mm: %.3f, Y mm: %.3f, H degree: %.3f}", iceWaddler.currentPos.getX(DistanceUnit.MM), iceWaddler.currentPos.getY(DistanceUnit.MM), iceWaddler.currentPos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position in inch", data);
            telemetry.addData("Position in mm", data2);
            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            telemetry.addLine("---------PinPoint Velocity X, Y ---------");
            String odoVel = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", iceWaddler.currentVel.getX(DistanceUnit.INCH), iceWaddler.currentVel.getY(DistanceUnit.INCH), iceWaddler.currentVel.getHeading(AngleUnit.RADIANS));
            String odoVel2 = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", iceWaddler.currentVel.getX(DistanceUnit.MM), iceWaddler.currentVel.getY(DistanceUnit.MM), iceWaddler.currentVel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity in inch", odoVel);
            telemetry.addData("Velocity in mm", odoVel2);

            /*
            Get Raw Tick Readings
             */
            telemetry.addLine("---------Encoder X, Y---------");
            telemetry.addData("Encoder X tick", iceWaddler.odo.getEncoderX());
            telemetry.addData("Encoder Y tick", iceWaddler.odo.getEncoderX());

            //Finds tick time in seconds, might move to IceWaddler
            double tickTime=getRuntime()-lastTickTimestamp;
            lastTickTimestamp=getRuntime();

            telemetry.addLine("---------Frequency--------");
            telemetry.addData("Pinpoint Frequency", iceWaddler.odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
            telemetry.addData("Tick Time", tickTime);

            if(iceWaddler.currentAction.actionType == IceWaddlerAction.ACTIONTYPE.PTP){
                telemetry.addLine("---------PTP Debug--------");
                telemetry.addData("latCorrection", iceWaddler.latCorrection);
                telemetry.addData("lonCorrection", iceWaddler.lonCorrection);
                telemetry.addData("rotCorrection", iceWaddler.rotCorrection);
                telemetry.addData("line angle", iceWaddler.lineAngle);
                telemetry.addData("TargetVelX", iceWaddler.targetVel.getX(DistanceUnit.METER));
                telemetry.addData("TargetVelY", iceWaddler.targetVel.getY(DistanceUnit.METER));
                telemetry.addData("PowerX", iceWaddler.targetPower.getX(DistanceUnit.METER));
                telemetry.addData("PowerY", iceWaddler.targetPower.getY(DistanceUnit.METER));
                telemetry.addData("TargetRot", iceWaddler.startingPos.getHeading(AngleUnit.RADIANS)+iceWaddler.actionCompletion*(iceWaddler.targetPos.getHeading(AngleUnit.RADIANS)-iceWaddler.startingPos.getHeading(AngleUnit.RADIANS)));
                telemetry.addData("CurrentRot", iceWaddler.currentPos.getHeading(AngleUnit.DEGREES));
            }

            //Log pose2D every second
            if(getRuntime()-lastPoseLog>1) {
                Pose2DLog.add(iceWaddler.currentPos);
                lastPoseLog=getRuntime();
            }

            //Write past positions to Dashboard Map
            TelemetryPacket fieldPacket = new TelemetryPacket();
            fieldPacket.put("x", 3.7);
            fieldPacket.put("status", "alive");

            for (int i = 1; i < Pose2DLog.size(); i++) {
                fieldPacket.fieldOverlay()
                    .setAlpha(1)
                    .setStroke("lime")
                    .setStrokeWidth(1)
                    .strokeLine(Pose2DLog.get(i-1).getX(DistanceUnit.INCH),Pose2DLog.get(i-1).getY(DistanceUnit.INCH),
                            Pose2DLog.get(i).getX(DistanceUnit.INCH),Pose2DLog.get(i).getY(DistanceUnit.INCH));
            }

            //Draw target line on dashboard
            if(iceWaddler.currentAction.actionType == IceWaddlerAction.ACTIONTYPE.PTP){
                fieldPacket.fieldOverlay()
                        .setAlpha(1)
                        .setStroke("light blue")
                        .setStrokeWidth(2)
                        .strokeLine(iceWaddler.startingPos.getX(DistanceUnit.INCH),iceWaddler.startingPos.getY(DistanceUnit.INCH),
                                iceWaddler.targetPos.getX(DistanceUnit.INCH),iceWaddler.targetPos.getY(DistanceUnit.INCH));
            }

            //Write current position and rotation to Dashboard Map
            double x = iceWaddler.currentPos.getX(DistanceUnit.INCH);
            double y = iceWaddler.currentPos.getY(DistanceUnit.INCH);
            double a = iceWaddler.currentPos.getHeading(AngleUnit.RADIANS);

            double[] xPoints = {
                    x+9*Math.cos(a)-9*Math.sin(a),
                    x+9*Math.cos(a)+9*Math.sin(a),
                    x-9*Math.cos(a)+9*Math.sin(a),
                    x-9*Math.cos(a)-9*Math.sin(a)
            };
            double[] yPoints = {
                    y+9*Math.sin(a)+9*Math.cos(a),
                    y+9*Math.sin(a)-9*Math.cos(a),
                    y-9*Math.sin(a)-9*Math.cos(a),
                    y-9*Math.sin(a)+9*Math.cos(a)
            };

            fieldPacket.fieldOverlay()
                    .setAlpha(1)
                    .setStroke("white")
                    .setStrokeWidth(1)
                    .strokePolygon(xPoints,yPoints);

            fieldPacket.fieldOverlay()
                    .setAlpha(0.5)
                    .setFill("white")
                    .fillPolygon(xPoints,yPoints);

            telemetry.update();
            dashboard.sendTelemetryPacket(fieldPacket);
        }
    }
}

