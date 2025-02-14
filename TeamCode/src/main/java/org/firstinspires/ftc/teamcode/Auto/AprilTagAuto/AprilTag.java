package org.firstinspires.ftc.teamcode.Auto.AprilTagAuto;

import android.util.Size;

import androidx.annotation.NonNull;

import java.util.HashMap; // import the HashMap class

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTag {

    private static AprilTagProcessor tagProcessor;
    private static VisionPortal visionPortal;
    private static HardwareMap hardwareMap;

    private static Integer[] aprilTagCoordinateArray = {null, null};
    private static Double[] tagInfo = {null, null, null, null};

    public AprilTag(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init () {
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


        visionPortal.resumeLiveView();
        visionPortal.resumeLiveView();

    }

    @NonNull
    public static HashMap<Integer, Integer[]> aprilTagMsg() {
        HashMap<String, Double> robotOffSet = new HashMap<>();
        robotOffSet.put("x", null);
        robotOffSet.put("y", null); /** Put the actual value later **/

        HashMap<Integer, Integer[]> aprilTagCoordinate = new HashMap<>();
        Integer[] aprilTagCoordinateArrayID11 = new Integer[]{-72, 48};
        Integer[] aprilTagCoordinateArrayID12 = new Integer[]{0, 72};
        Integer[] aprilTagCoordinateArrayID13 = new Integer[]{72, 48};
        Integer[] aprilTagCoordinateArrayID14 = new Integer[]{72, -48};
        Integer[] aprilTagCoordinateArrayID15 = new Integer[]{0, -72};
        Integer[] aprilTagCoordinateArrayID16 = new Integer[]{-72, -48};

        aprilTagCoordinate.put(11, aprilTagCoordinateArrayID11);
        aprilTagCoordinate.put(12, aprilTagCoordinateArrayID12);
        aprilTagCoordinate.put(13, aprilTagCoordinateArrayID13);
        aprilTagCoordinate.put(14, aprilTagCoordinateArrayID14);
        aprilTagCoordinate.put(15, aprilTagCoordinateArrayID15);
        aprilTagCoordinate.put(16, aprilTagCoordinateArrayID16);

        return aprilTagCoordinate;
    }

    private Integer[] aprilTagUpdate () {
        if (!tagProcessor.getDetections().isEmpty()) {

            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            aprilTagCoordinateArray = aprilTagMsg().get(tag.id);

        }
        return aprilTagCoordinateArray;
    }

    private Double[] tagAxis () {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);

            tagInfo[0] = tag.ftcPose.x;
            tagInfo[1] = tag.ftcPose.y;
            tagInfo[2] = tag.ftcPose.bearing;
            tagInfo[3] = tag.ftcPose.yaw;
        }
        return tagInfo;
    }


    //public static double getPose () {
        //return
    //}
}
