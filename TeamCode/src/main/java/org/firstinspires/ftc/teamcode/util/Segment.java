package org.firstinspires.ftc.teamcode.util;


import org.firstinspires.ftc.teamcode.robot.BasicBot;

import java.util.Locale;

public class Segment
{
    public Segment(String name, Point2d start, Point2d end)
    {
        this.name   = name;
        this.strtPt = start;
        this.tgtPt  = end;
        this.fldHdg = angle();
        this.postTurn = fldHdg;
        len = start.distance(end);
        this.dir = BasicBot.DriveDir.INTAKE;
        this.speed = DEF_SEG_SPD;
        this.act = Action.NOTHING;
        this.drvTuner = 1.0;
        this.tgtType = TargetType.ENCODER;
    }

    public BasicBot.DriveDir getDir()
    {
        return dir;
    }
    public double getFieldHeading()
    {
        return fldHdg;
    }

    public String getName()
    {
        return name;
    }
    public Point2d getStrtPt()
    {
        return strtPt;
    }
    public Point2d getTgtPt()
    {
        return tgtPt;
    }
    public double getSpeed() { return speed; }
    public Action getAction() { return act; }
    public double getLength() { return len; }
    public double getDrvTuner() { return drvTuner; }
    public double getPostTurn() { return postTurn; }
    public TargetType getTgtType() { return tgtType; }

    public void setName(String name) {this.name = name;}
    public void setAction(Action act) { this.act = act; }
    public void setDir(BasicBot.DriveDir dir)
    {
        this.dir = dir;
    }
    public void setSpeed(double spd) { this.speed = spd; }
    public void setDrvTuner(double drvTuner) { this.drvTuner = drvTuner; }
    public void setPostTurn(double postTurn) { this.postTurn = postTurn; }
    public void setStrtPt(Point2d spt)
    {
        this.strtPt = spt;
        this.fldHdg = angle();
        len = strtPt.distance(tgtPt);
    }
    public void setEndPt(Point2d ept)
    {
        this.tgtPt = ept;
        this.fldHdg = angle();
        len = strtPt.distance(tgtPt);
    }
    public void setTgtType(TargetType tgtType)
    {
        this.tgtType = tgtType;
    }

    public double angle()
    {
        double tgtFldHdg = Math.atan2(tgtPt.getY() - strtPt.getY(), (tgtPt.getX() - strtPt.getX()));
        return Math.toDegrees(tgtFldHdg);
    }

    public String toString()
    {
        return String.format(Locale.US, "%s: %s - %s len: %5.2f %s hdg: %5.2f " +
                                        " spd: %.3f act: %s post: %5.2f",
                name, strtPt, tgtPt, len, dir.toString(), fldHdg, speed, act.toString(), postTurn);
    }

    //enum SegDir {FORWARD, REVERSE}
    public enum Action {NOTHING, SHOOT, SCAN_IMAGE, FIND_BEACON, PUSH,
        RST_PUSHER, SET_KEY, DROP, GRAB, PREGRAB, SET_ALIGN, RETRACT, ESCAPE, THROW, PARK}
    public enum TargetType{ENCODER, TIME, COLOR}
    private static final double DEF_SEG_SPD = 0.5;
    private double  fldHdg;
    private Point2d strtPt;
    private Point2d tgtPt;
    private String  name;
    private BasicBot.DriveDir dir;
    private double speed;
    private double len;
    private Action act;
    private double drvTuner;
    private double postTurn;
    private TargetType tgtType;
}
