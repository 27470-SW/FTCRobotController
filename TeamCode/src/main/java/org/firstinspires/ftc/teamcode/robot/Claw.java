package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;



public class Claw {

    public Claw(HardwareMap map)
    {
        this.hwMap = map;
        clawServo = hwMap.get(Servo.class, "clawservo");
        percent = 1;
        maxOpen = 0;
        maxClosed = 1;

    }

    public void setClawPos(double pos)
    {
        pos = 1-pos;
        if(clawServo == null) return;
        clawServo.setPosition(pos);
<<<<<<< Updated upstream
    }

=======
        RobotLog.dd(TAG, "Percent:%f POS:%f", percent, pos);

    }
/*
    public void setClawLev(double lev)
    {
        if (lev == 0){
            setClawPos(0);
        }
        if (lev == 1){
            setClawPos(.6);
        }
        if (lev == 2){
            setClawPos(1);
        }
        
    }*/
>>>>>>> Stashed changes
public void update(){
        setClawPos(percent);
}

    public void openClaw(double val){
<<<<<<< Updated upstream
        percent = percent - .007*val;
=======
        RobotLog.dd(TAG, "Percent:%f Val:%f", percent, val);

        percent = percent - .07*val;
>>>>>>> Stashed changes
        if (percent > maxClosed)
        {
            percent = maxClosed;
        }
        setClawPos(percent);
    }

    public void closeClaw(double val){
<<<<<<< Updated upstream
        percent = percent + .007*val;
=======
        RobotLog.dd(TAG, "Percent:%f Val:%f", percent, val);

        percent = percent + .07*val;
>>>>>>> Stashed changes
        if (percent > maxClosed)
        {
            percent = maxClosed;
        }
        setClawPos(percent);
    }

    /*
    public void clawFunctions(double var){
        RobotLog.dd(TAG, "var: %f K1R2: %d Triggr: %b, percent:%f", var, K1R2, Triggr, percent);
        if (Triggr == true){
            if (K1R2 == 1)
            {
            openClaw(var);
            }
            if (K1R2 == 2){
            closeClaw(var);
            }
        } if (Triggr == false){
            setClawLev(var);
        }
    }
     */


    public boolean init(){
        boolean success;

        try{
            clawServo = hwMap.get(Servo.class, "camservo");
            success = true;
            setClawPos(RobotConstants.CAM_STOW);
            if(clawServo instanceof ServoImplEx)
            {
                ServoImplEx sie = ((ServoImplEx) clawServo);
                PwmControl.PwmRange rng = sie.getPwmRange();
                RobotLog.dd(TAG, "Camservo pwm range: %.2f to %.2f",
                        rng.usPulseLower, rng.usPulseUpper);
            }
        }catch (Exception e)
        {
            RobotLog.ee(TAG, "ERROR: Cam Servo missing");
            success = false;
        }
        return success;
    }
    public double percent;
    private static final String TAG = "claw";
    public Servo clawServo;
    protected HardwareMap hwMap;
    public double maxOpen;
    public double maxClosed;
    public boolean Triggr;
    public int K1R2;
}
