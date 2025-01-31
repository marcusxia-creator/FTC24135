
package org.firstinspires.ftc.teamcode.Auto.Roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Auto.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@Autonomous(name="RightSideAuto", group="org.firstinspires.ftc.teamcode.AutoTest.Roadrunner")
@Config
public class RightSideAuto extends LinearOpMode {

    public static double highbar_x_coordinate = 0;
    public static double highbar_y_coordinate = -32;
    public static double specimen_pickup_x_coordinate = 24;
    public static double specimen_pickup_y_coordinate = -48;
    public static double first_sample_pickup_x_coordinate = 36;
    public static double first_sample_pickup_y_coordinate = -36;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RobotActionConfig value = new RobotActionConfig();
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.intakeWristServo.setPosition(0.3);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);
        robot.intakeLeftSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);
        robot.intakeRightSlideServo.setPosition(RobotActionConfig.intake_Slide_Retract);

        Pose2d startPose = new Pose2d(7.5, -64, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(first_sample_pickup_x_coordinate,first_sample_pickup_y_coordinate,Math.toRadians(45)))
                .lineToLinearHeading(new Pose2d(first_sample_pickup_x_coordinate,first_sample_pickup_y_coordinate,Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(specimen_pickup_x_coordinate,specimen_pickup_y_coordinate,Math.toRadians(-45)))
                .lineToLinearHeading(new Pose2d(highbar_x_coordinate,highbar_y_coordinate,Math.toRadians(-90)))
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }

    private void Slides_Move(int dist, double speed) {
        RobotHardware robot = new RobotHardware();
        robot.init(hardwareMap);

        robot.liftMotorLeft.setTargetPosition(dist);
        robot.liftMotorRight.setTargetPosition(dist);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        while (opModeIsActive() && (robot.liftMotorLeft.isBusy() && robot.liftMotorRight.isBusy())) {}
    }
}