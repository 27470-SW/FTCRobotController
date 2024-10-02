package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

public class Lifter
{
    public Lifter(String cfgName, HardwareMap map)
    {
        this.name  = cfgName;
        this.hwMap = map;
    }

    public boolean init()
    {
        boolean success = false;
        try
        {
            liftMotor = hwMap.get(DcMotorEx.class, name);
            liftMotor.setDirection(RobotConstants.LF_ARM_ROT_DIR);
            setLiftPwr(0.0);
            //setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(RUN_USING_ENCODER);
            RobotLog.dd(TAG, "Lifter.init Found liftMotor " + name);
            success = true;
        }
        catch (Exception ignored)
        {
        }
        try
        {
            liftServo = hwMap.get(CRServo.class, name);
            liftServo.setDirection(RobotConstants.LF_ARM_ROT_DIR);
            setLiftPwr(0.0);
            RobotLog.dd(TAG, "Lifter.init Found liftServo " + name);
            success = true;
        }
        catch (Exception ignored)
        {
        }

        if(!success) RobotLog.ee(TAG, "ERROR lifter - no liftMotor or liftServo " + name);

        return success;
    }

    //pos in counts
    public void setLiftPos(int pos)
    {
        tgtCnt = pos;
        if (liftMotor != null) liftMotor.setTargetPosition(pos);

        if (lastRunMode != RUN_TO_POSITION)
        {
            setMode(RUN_TO_POSITION);
        }

        if (liftMotor != null)
        {
            setLiftPwr(RobotConstants.LF_ARM_ROT_SPD);
        }
    }

    public void setLiftSpd(double pwr)
    {
        if (Math.abs(pwr) < 0.05 && lastRunMode != RUN_TO_POSITION)
        {
            setLiftPos(lftCnts);
        }
        else if (Math.abs(pwr) >= 0.05)
        {
            if (lastRunMode != RUN_USING_ENCODER)
            {
                setMode(RUN_USING_ENCODER);
            }

            //safetycheck
            //TODO: replace with endstop check
            double lftmin = Math.min(RobotConstants.LF_ARM_ROT_STOW, RobotConstants.LF_ARM_ROT_LOW);
            double lftmax = RobotConstants.LF_ARM_ROT_MAX;
            if (lftCnts <= lftmin && pwr < 0.0 ||
                lftCnts >= lftmax  && pwr > 0.0) pwr = 0.0;


            if (liftMotor != null)
            {
                setLiftPwr(pwr * RobotConstants.LF_ARM_ROT_SPD);
            }
        }
    }

    public void setMode(DcMotor.RunMode mode)
    {
        if (liftMotor != null && mode != lastRunMode)
        {
            liftMotor.setMode(mode);
            lastRunMode = mode;
        }
    }

    public void setLiftPwr(double pwr)
    {
        if(cmdPwr == pwr) return;
        if (liftMotor != null) liftMotor.setPower(pwr);
        if (liftServo != null) liftServo.setPower(pwr);
        cmdPwr = pwr;

        //Could use a pwr*maxAllowedDps and the LIFTER_CPD  to get CPS then use setVelocity
    }

    public void update()
    {
        if(liftMotor != null) lftCnts = liftMotor.getCurrentPosition();
    }

    public String toString()
    {
        return String.format(Locale.US,
                "lift %5d %5d %.2f %s",
                lftCnts, tgtCnt, cmdPwr, lastRunMode);
    }

    private DcMotorEx liftMotor;
    private CRServo liftServo;
    protected HardwareMap hwMap;
    protected final String name;

    private DcMotor.RunMode lastRunMode;
    private int tgtCnt = 0;
    private int lftCnts = 0;
//    private double tgtVel = 0.0;
    private double cmdPwr = Double.MAX_VALUE;

    private static final double LIFTER_CPER = 28; //quad encoder cnts/encoder rev
    private static final Motors.MotorModel mod = Motors.MotorModel.GOBILDA_5202_71_2;
    private static final double LIFTER_INT_GEAR = mod.getGear();
    private static final double LIFTER_EXT_GEAR = RobotConstants.LF_ARM_ROT_GEAR;
    private static final double LIFTER_CPR = LIFTER_CPER * LIFTER_INT_GEAR * LIFTER_EXT_GEAR; // cnts/outShaftRev
    private static final double LIFTER_CPD = LIFTER_CPR / 360.0;
    private static final int    LIFTER_OFST = (int)(RobotConstants.LF_STOW_DEG * LIFTER_CPD);

    private static final String TAG = "SJH_LFT";
}
