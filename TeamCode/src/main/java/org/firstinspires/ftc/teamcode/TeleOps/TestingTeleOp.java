package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestingTeleOp extends OpMode {
    private AutoVisionProcessing autoVisionProcessing;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        autoVisionProcessing = new AutoVisionProcessing(dashboard ,hardwareMap);
        autoVisionProcessing.initialize();

        telemetry.addLine("Opening_Camera...");
        telemetry.update();
    }

    @Override
    public void loop() {
        autoVisionProcessing.process(false);

        if (autoVisionProcessing.sampleX != 0.0) {
            telemetry.addData("Real X", autoVisionProcessing.sampleX);
            telemetry.addData("Real Y", autoVisionProcessing.sampleY);
            telemetry.addData("Angles", autoVisionProcessing.sampleAngles);
        } else {
            telemetry.addLine("No Sample Detected");
        }
    }
}
