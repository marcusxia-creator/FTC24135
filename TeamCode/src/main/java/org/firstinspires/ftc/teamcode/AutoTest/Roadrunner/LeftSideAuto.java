package org.firstinspires.ftc.teamcode.AutoTest.Roadrunner;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.AutoTest.Roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;


@Autonomous(name="LeftSideAuto", group="org.firstinspires.ftc.teamcode.AutoTest.Roadrunner")
@Config
public class LeftSideAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    static final double COUNTS_PER_MOTOR_GOBILDA_435 = 384.5;
    static final double COUNTS_PER_MOTOR_GOBILDA_312 = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.5; //24:16 Motor:Wheel
    static final double WHEEL_DIAMETER_MM = 96; // Wheel diameter mm
    static final double COUNTS_PER_MM_Drive = (COUNTS_PER_MOTOR_GOBILDA_435 * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);
    static final double COUNTS_PER_CM_Slides = COUNTS_PER_MOTOR_GOBILDA_312 / 38.2; //Ticks Per Rotation * Pulley Circumference

    public static double powerFactor = 1;

    //Intake Configure


    public static double deposit_Wrist_Dump_Pos = 0.22; //range(0-1), 0: installation position
    public static double deposit_Wrist_Transfer_Pos = 0.52; // 0.06 is rest position.

    public static double deposit_Arm_Dump_Pos = 0.7; // range (0-1) 0: installation position 180 deg
    public static double deposit_Arm_Transfer_Pos = 0.05; // 0 is rest position.

    public static double deposit_Claw_Open = 0.36;  //
    public static double deposit_Claw_Close = 0.0;

    public static double dumpTime = 0.5; // deposit time need to rotate deposit arm then open claw

    //movement positions
    public static double basket_x_coordinate = -59;
    public static double basket_y_coordinate = -59;
    public static double first_sample_x_coordinate = -48;
    public static double first_sample_y_coordinate = -38;
    public static double second_sample_x_coordinate = -58;
    public static double second_sample_y_coordinate = -48;
    public static double third_sample_x_coordinate = -58;
    public static double third_sample_y_coordinate = -48;
    public static double third_sample_heading = 114;

    public static final double SLIDES_MOVE_HEIGHT = 3226;
    public static final double SLIDES_MOVE_POWER = 0.9;

    public ElapsedTime timer = new ElapsedTime();

    float timer_log;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        robot.init(hardwareMap);

        robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);
        robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);
        robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);
        robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);
        robot.intakeRotationServo.setPosition(RobotActionConfig.intake_Rotation_Mid);
        robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);
        robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);

        Pose2d startPose = new Pose2d(-40, -64.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(3226,0.9);})
                .waitSeconds(1.8)                                                                                             // need to review if this wait time is necessary?
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Dump);})                          // start after wait seconds
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Dump);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Open);})
                .waitSeconds(0.5)                                                                                          // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(RobotActionConfig.deposit_Arm_Transfer);})                      // at global time 1.5 second mark to back to transfer position
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(RobotActionConfig.deposit_Wrist_Transfer);})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Slides_Move(50,0.9);})                             // at global time 1.5 second mark to back to lower slides.
                .waitSeconds(3.0)                                                                                              // wait time is count from previous addTemporalmarker it needs to account for slides moving down time. the next sequence.
                /** move to 1st sample*/
                .lineToLinearHeading(new Pose2d(first_sample_x_coordinate,first_sample_y_coordinate,Math.toRadians(90)))
                /** pick 2nd sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);})
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Pick);})
                .addTemporalMarker(()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Pick);})
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Close);})
                .waitSeconds(0.5)
                /**transfer 2nd sample*/
                .addTemporalMarker(()->{robot.intakeLeftArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);})
                .addTemporalMarker(()->{robot.intakeRightArmServo.setPosition(RobotActionConfig.intake_Arm_Transfer);})
                .addTemporalMarker(()->{robot.intakeWristServo.setPosition(RobotActionConfig.intake_Wrist_Transfer);})
                .UNSTABLE_addTemporalMarkerOffset(0.9,()->{robot.depositClawServo.setPosition(RobotActionConfig.deposit_Claw_Close);})
                .UNSTABLE_addTemporalMarkerOffset(1.0,()->{robot.intakeClawServo.setPosition(RobotActionConfig.intake_Claw_Open);})
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))
                /** drop 2nd sample*/
                .UNSTABLE_addTemporalMarkerOffset(0,()->{drive.setDrivePower(new Pose2d(0,0,0));})
                .UNSTABLE_addTemporalMarkerOffset(0,()->{Slides_Move(3226,0.9);})
                .waitSeconds(1.8)                                                                                             // need to review if this wait time is necessary?
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(deposit_Arm_Dump_Pos);})                          // start after wait seconds
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(deposit_Wrist_Dump_Pos);})
                .waitSeconds(0.4)
                .addTemporalMarker(()->{robot.depositClawServo.setPosition(deposit_Claw_Open);})
                .waitSeconds(0.5)                                                                                          // this is wait time for dropping sample - wait for open claw to drop
                .addTemporalMarker(()->{robot.depositArmServo.setPosition(deposit_Arm_Transfer_Pos);})                      // at global time 1.5 second mark to back to transfer position
                .addTemporalMarker(()->{robot.depositWristServo.setPosition(deposit_Wrist_Transfer_Pos);})
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{Slides_Move(50,0.9);})                             // at global time 1.5 second mark to back to lower slides.
                .waitSeconds(3.0)
                /** move to 3rd sample */
                .lineToLinearHeading(new Pose2d(second_sample_x_coordinate,second_sample_y_coordinate,Math.toRadians(90)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(third_sample_x_coordinate,third_sample_y_coordinate,Math.toRadians(third_sample_heading)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(basket_x_coordinate,basket_y_coordinate,Math.toRadians(45)))
                .waitSeconds(4)
                .build();

        waitForStart();
        timer.reset();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }

    }

    private void Slides_Move(int dist, double speed) {
        robot.liftMotorLeft.setTargetPosition(dist);
        robot.liftMotorRight.setTargetPosition(dist);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotorLeft.setPower(speed);
        robot.liftMotorRight.setPower(speed);
        while (opModeIsActive() && (robot.liftMotorLeft.isBusy() && robot.liftMotorRight.isBusy())) {

        }
        sleep(200);
    }
}
