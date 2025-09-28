package org.firstinspires.ftc.teamcode.Auto.IceWaddlerPaths;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IceWaddler;

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

    public static List<IceWaddler.Action> Path = Arrays.asList(
            new IceWaddler.Action(startingPose, intakePose, true),
            new IceWaddler.Action(IceWaddler.Action.HOLDTYPE.POS, intakeExtendDelay),
            new IceWaddler.Action(IceWaddler.Action.HOLDTYPE.POS, intakePickDelay),
            new IceWaddler.Action(intakePose, depositPose, true),
            new IceWaddler.Action(IceWaddler.Action.HOLDTYPE.POS),
            new IceWaddler.Action(depositPose, endingTransistionPose, false),
            new IceWaddler.Action(endingTransistionPose, endingPose, true)
    );


    public static double intakeExtendDistance = 0.6;
}
