package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotActionConfig {
    //drive chassis
    public static double powerFactor =0.7;
    /**Intake Config**/
    public static double intake_Slide_Extend = 0.3;// range(0 - 0.3)
    public static double intake_Slide_Retract = 0;

    public static double intake_Arm_Idle = 0.25;
    public static double intake_Arm_Pick = 0.41;
    public static double intake_Arm_Transfer = 0.1;
    public static double intake_Arm_Change_Amount = 0.05;

    public static double intake_Wrist_Idle = 0.64;
    public static double intake_Wrist_Pick = 0.64;
    public static double intake_Wrist_Transfer = 0.07;

    public static double intake_Rotation_Idle = 0.46;
    public static double intake_Rotation_Steer_Amount = 0.1;

    public static double intake_Claw_Open = 0;
    public static double intake_Claw_Close = 0.27;



    //Deposit Slide Config
    public static int deposit_Slide_down_Pos = 50;   //slides Position Configure
    public static int deposit_Slide_Highbar_Up_Pos = 525;//slides Position Configure
    public static int deposit_Slide_Highbasket_Pos = 3200; //slides Position Configure
    public static double deposit_Slide_UpLiftPower = 0.9;  //slides power
    public static double deposit_Slide_DownLiftPower = 0.3;

    //Deposit Wrist config
    public static double deposit_Wrist_dump_Pos = 0.22;
    public static double deposit_Wrist_Transfer_Pos = 0.53;
    public static double deposit_Wrist_Highbar_Pos = 0.14;
    public static double deposit_Wrist_PickUp_Pos = 0.31;

    //deposit arm config
    public static double deposit_Arm_dump_Pos = 0.76;
    public static double deposit_Arm_Transfer_Pos = 0.06;
    public static double deposit_Arm_hang_Pos = 0.8;
    public static double deposit_Arm_Highbar_Pos = 0.76;
    public static double deposit_Arm_PickUp_Pos = 0.97;

    //deposit claw config
    public static double deposit_Claw_Open = 0.4;
    public static double deposit_Claw_Close = 0.0;

    public static float hsvValues[] = {0F,0F,0F};
    //Time
    public static double intake_Slide_Retract_Threshold = 0.1;
    public static double intake_Wrist_Arm_Retract_Threshold = 0.5;
    public static double deposit_Claw_Close_Threshold = 1;
    public static double intake_Claw_Open_Threshold = 1.3;
    public static double intake_Arm_Idle_Threshold = 1.5;

    public static double DEBOUNCE_THRESHOLD = 0.25;
}
