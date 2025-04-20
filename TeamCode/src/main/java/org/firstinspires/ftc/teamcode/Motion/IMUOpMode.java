package org.firstinspires.ftc.teamcode.Motion;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

@TeleOp(name = "IMU Smoothed Heading Demo", group = "Sensor")
public class IMUOpMode extends LinearOpMode {

    private RobotHardware robot;
    private IMUReader imuReader;

    @Override
    public void runOpMode() {
        // --- IMU Initialization ---
        // Initialize hardware in RobotHardware
        robot = new RobotHardware();
        robot.init(hardwareMap);
        robot.initIMU();


        telemetry.addData("Status", "IMU initialized");
        telemetry.update();

        // --- Start IMUReader with buffer size of 20 samples (~100ms window) ---
        imuReader = new IMUReader(robot, 20);
        imuReader.start();

        waitForStart();

        while (opModeIsActive()) {
            double smoothHeading = imuReader.getAverageHeading();
            telemetry.addData("Smoothed Heading", "%.2f°", smoothHeading);
            telemetry.update();

            sleep(50); // poll every 50ms
        }

        // Stop the background thread
        imuReader.stop();
    }
}
