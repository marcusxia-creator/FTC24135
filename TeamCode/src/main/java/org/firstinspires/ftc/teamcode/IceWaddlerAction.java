package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class IceWaddlerAction {
    //Common variables
    public ACTIONTYPE actionType;
    public boolean holdVel;

    public enum ACTIONTYPE{
        DELAY,
        BOOL,
        PTP
    }
    //Delay Variables
    public double DelayLength;

    //PTP variables
    public Pose2D startingPos;
    public Pose2D targetPos;
    public boolean decelerate;

    public IceWaddlerAction(double DelayLength, boolean holdVel){
        this.actionType = ACTIONTYPE.DELAY;
        this.DelayLength = DelayLength;
        this.holdVel=holdVel;
    }

    public IceWaddlerAction(boolean holdVel){
        this.actionType = ACTIONTYPE.BOOL;
        this.holdVel=holdVel;
    }

    public IceWaddlerAction(Pose2D startingPos, Pose2D targetPos, boolean decelerate){
        this.actionType = ACTIONTYPE.PTP;
        this.startingPos = startingPos;
        this.targetPos = targetPos;
        this.decelerate = decelerate;
    }
}