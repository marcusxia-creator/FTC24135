package org.firstinspires.ftc.teamcode.TeleOps.ApirlTag;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp (name= "April_Tag_3d_Post_Estimation_TEST", group = "org/firstinspires/ftc/teamcode/OpMode")
public class AprilTagVisionPortalTest extends OpMode {

    private static AprilTagProcessor tagProcessor;
    private static VisionPortal visionPortal;

    @Override
    public void init (){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Web_Cam"))
                .setCameraResolution(new Size(640, 480))
                .build();



    }

    @Override
    public void loop() {

        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            visionPortal.resumeLiveView();
            visionPortal.resumeLiveView();

            telemetry.addData("Bearing", tag.ftcPose.bearing);
            telemetry.addData("x", tag.ftcPose.x);
            telemetry.addData("y", tag.ftcPose.y);
            telemetry.addData("z", tag.ftcPose.z);
            telemetry.addData("Roll", tag.ftcPose.roll);
            telemetry.addData("Pitch", tag.ftcPose.pitch);
            telemetry.addData("Yaw", tag.ftcPose.yaw);
            telemetry.addData("Range", tag.ftcPose.range);
        }

        telemetry.addData("April tag is detected", !tagProcessor.getDetections().isEmpty());

        telemetry.update();
    }
}
