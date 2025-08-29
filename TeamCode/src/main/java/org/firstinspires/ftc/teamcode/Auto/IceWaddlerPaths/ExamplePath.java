package org.firstinspires.ftc.teamcode.Auto.IceWaddlerPaths;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddlerAction;

import java.util.Arrays;
import java.util.List;

@Config
public class ExamplePath {
    public static Pose2D startingPose = new Pose2D(DistanceUnit.METER, 0 ,0, AngleUnit.DEGREES, 0);

    public static Pose2D intakePose = new Pose2D(DistanceUnit.METER, 0.5,1, AngleUnit.DEGREES, 90);

    public static double intakeExtendDelay = 0.2;
    public static double intakePickDelay = 0.8;

    public static Pose2D depositPose = new Pose2D(DistanceUnit.METER, 0.3,-0.5, AngleUnit.DEGREES, 45);

    public static Pose2D endingTransistionPose = new Pose2D(DistanceUnit.METER, 0.5,-0.5, AngleUnit.DEGREES, 0);
    public static Pose2D endingPose = new Pose2D(DistanceUnit.METER, 1,0, AngleUnit.DEGREES, -90);

    public static List<IceWaddlerAction> Path = Arrays.asList(
            new IceWaddlerAction(startingPose, intakePose, true),
            new IceWaddlerAction(intakeExtendDelay, true),
            new IceWaddlerAction(intakePickDelay, true),
            new IceWaddlerAction(intakePose, depositPose, true),
            new IceWaddlerAction(true),
            new IceWaddlerAction(depositPose, endingTransistionPose, false),
            new IceWaddlerAction(endingTransistionPose, endingPose, true)
    );

    public static double intakeExtendDistance = 0.6;
}
