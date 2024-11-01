package org.firstinspires.ftc.teamcode.test;

import android.util.SparseArray;
import android.util.SparseBooleanArray;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.opModes.InitLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Motors;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.util.ManagedGamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

/**
 * This OpMode steps n motors speed up and down based on D-Pad user inputs.
 * This code assumes a DC motor configured with the name "testmotor#".
 * If motors are on a bot, left side motors should be odd and right side even
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 */
@TeleOp(name = "TestMotors", group = "Test")
//@Disabled
public class StepMotorTest extends InitLinearOpMode
{
    private static final double INCREMENT = 0.02;     // amount to step motor each CYCLE_MS cycle
    private static final int     CYCLE_MS = 50;       // period of each cycle
    private static final double   MAX_FWD =  1.0;     // Maximum FWD power applied to motor
    private static final double   MAX_REV = -1.0;     // Maximum REV power applied to motor

    // Define class members
    private static final DcMotor.Direction LFT_DIR  = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction RGT_DIR  = DcMotor.Direction.FORWARD;

    private static final int MAX_MOTORS = 4;

    private double power = 1;
    private final ArrayList<Double> motPwrs = new ArrayList<>(MAX_MOTORS);

    public static Motors.MotorModel DT_MOTOR = Motors.MotorModel.GOBILDA_5202_19_2;
    private final double MAX_CPS = DT_MOTOR.getCpr() * DT_MOTOR.getRpm() / 60.0;
    private boolean useSpd = false;

    private static final String TAG = "SJH_RMT";
    protected static final boolean VERBOSE = true;

    @Override
    public void runOpMode()
    {
        initCommon(this);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        LynxModule.BulkCachingMode bcm = LynxModule.BulkCachingMode.AUTO;
        String bcmStr = "A";
        for (LynxModule module : allHubs)
        {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        double strtV = getBatteryVoltage();

        SparseArray<DcMotorEx> motors = new SparseArray<>(MAX_MOTORS);
        SparseBooleanArray   actLst = new SparseBooleanArray(MAX_MOTORS);

        int p;

        String motorNames[]={"FL","FR","BL","BR"};

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            String motorName =  motorNames[m];
            DcMotorEx mot;
            try
            {
                mot = hardwareMap.get(DcMotorEx.class, motorName);

                PIDFCoefficients pid;
                pid = mot.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
                if(VERBOSE){RobotLog.dd(TAG, "RUN_TO_POS Motor %s PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                        motorName, pid.p, pid.i, pid.d, pid.f);}
                pid = mot.getPIDFCoefficients(RUN_USING_ENCODER);
                if(VERBOSE){RobotLog.dd(TAG, "RUN_USING_ENC Motor %s PIDs. P:%.2f I:%.2f D:%.2f F:%.2f",
                        motorName, pid.p, pid.i, pid.d, pid.f);}
                if(VERBOSE){RobotLog.dd(TAG, "Found motor " + motorName);}
                PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                        0,0,0,0
                );
                mot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, compensatedCoefficients);

            }
            catch(IllegalArgumentException e)
            {
                if(VERBOSE){RobotLog.ee(TAG, "Problem finding motor " + motorName);}
                continue;
            }

            motors.put(m, mot);
            actLst.put(m, true);
            motPwrs.add(0.0);

            if(m%2 == 0) mot.setDirection(LFT_DIR);
            else         mot.setDirection(RGT_DIR);
            mot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mot.setMode(RUN_USING_ENCODER);
            mot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Wait for the start button
        dashboard.displayText(0, "Press Start to run Motors.");
        ElapsedTime timer = new ElapsedTime();
        double t1;
        double t2;
        double t3;
        double dt;
        double dtg;
        double dts;
        double ttg = 0.0;
        double tts = 0.0;
        double tt = 0.0;
        double at;
        int encPos;
        double motVel;
        int numCycles = 0;
        while(!isStarted())
        {
            p=0;

            gpad1.update();
            boolean toggle_bcm = gpad1.just_pressed(ManagedGamepad.Button.L_STICK_BUTTON);
            if(toggle_bcm)
            {
                if(bcm == LynxModule.BulkCachingMode.AUTO)
                {
                    bcm = LynxModule.BulkCachingMode.OFF;
                    bcmStr = "O";
                }
                else
                {
                    bcm = LynxModule.BulkCachingMode.AUTO;
                    bcmStr = "A";
                }
                for (LynxModule module : allHubs)
                {
                    module.setBulkCachingMode(bcm);
                }

                numCycles = 0;
                tt = 0;
            }

            for(int m = 0; m < MAX_MOTORS; m++)
            {
                DcMotorEx mot = motors.get(m);
                if(mot != null)
                {
                    t1 = timer.milliseconds();
                    encPos = mot.getCurrentPosition();
                    t2 = timer.milliseconds();
                    dt = t2 - t1;
                    dashboard.displayText(m,
                        String.format(Locale.US, "CNT_%d %d encTime=%4.3f", m, encPos, dt));
                    tt += dt;
                }
            }
            at = tt / ++numCycles;
            dashboard.displayText(MAX_MOTORS + p++,
                String.format(Locale.US, "AvgEncTime %4.3f", at));
            dashboard.displayText(MAX_MOTORS + p,"Tgl Blk mode: L plunge");

            sleep(10);
        }
        waitForStart();

        boolean tact = false;
        boolean strtTmr;
        int e0 = 0;
        int denc = 0;
        int ntc = 0;
        double tv = 0.0;
        double av = 0.0;
        double t0 = 0.0;
        double dtt = 0.0;
        double cps = 0.0;
        ElapsedTime bumpTimer = new ElapsedTime();
        double lastBumpTime = 0.0;
        double maxV = 0.0;

        int lScl = 0;
        int rScl = 0;

        while(opModeIsActive())
        {
            p=0;
            numCycles = 0;
            gpad1.update();
            boolean fwd        = gpad1.just_pressed(ManagedGamepad.Button.D_UP);
            boolean rev        = gpad1.just_pressed(ManagedGamepad.Button.D_DOWN);
            boolean ltrn       = gpad1.just_pressed(ManagedGamepad.Button.D_LEFT);
            boolean rtrn       = gpad1.just_pressed(ManagedGamepad.Button.D_RIGHT);
            boolean lstr       = gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X) < -0.5;
            boolean rstr       = gpad1.value(ManagedGamepad.AnalogInput.L_STICK_X) > 0.5;

            boolean tglAct0    = gpad1.just_pressed(ManagedGamepad.Button.A);
            boolean tglAct1    = gpad1.just_pressed(ManagedGamepad.Button.B);
            boolean tglAct2    = gpad1.just_pressed(ManagedGamepad.Button.X);
            boolean tglAct3    = gpad1.just_pressed(ManagedGamepad.Button.Y);

            boolean incrPwr    = gpad1.just_pressed(ManagedGamepad.Button.R_BUMP);
            boolean decrPwr    = gpad1.just_pressed(ManagedGamepad.Button.L_BUMP);
            boolean toggle_tmr = gpad1.just_pressed(ManagedGamepad.Button.L_STICK_BUTTON);
            boolean toggle_dir = gpad1.just_pressed(ManagedGamepad.Button.R_STICK_BUTTON);
            boolean toggle_mod = gpad1.just_pressed(ManagedGamepad.Button.L_TRIGGER);
            boolean toggle_spd = gpad1.just_pressed(ManagedGamepad.Button.R_TRIGGER);

            strtTmr = false;
            if(toggle_tmr)
            {
                tact = !tact;
                strtTmr = tact;
            }

            if(toggle_mod)
            {
                for(int m = 0; m < MAX_MOTORS; m++)
                {
                    if(!actLst.get(m)) continue;
                    DcMotor.RunMode newMode = RUN_USING_ENCODER;
                    DcMotor.RunMode curMode = motors.get(m).getMode();
                    if(curMode == RUN_USING_ENCODER)
                        newMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                    
                    DcMotorEx mot = motors.get(m);
                    if(mot != null) mot.setMode(newMode);
                }
            }

            if(toggle_dir)
            {
                for(int m = 0; m < MAX_MOTORS; m++)
                {
                    if(!actLst.get(m)) continue;
                    DcMotorEx mot = motors.get(m);
                    if(mot != null)
                    {
                        DcMotor.Direction dir = mot.getDirection();
                        if(dir == DcMotorSimple.Direction.FORWARD)
                            dir = DcMotorSimple.Direction.REVERSE;
                        else
                            dir = DcMotorSimple.Direction.FORWARD;
                        mot.setDirection(dir);
                    }
                }
            }

            if(toggle_spd) useSpd = !useSpd;

            double incDecScl = 1.0;
            if (incrPwr)
            {
                double timeNow = bumpTimer.seconds();
                if (timeNow - lastBumpTime < 0.5) incDecScl = 10.0;
                lastBumpTime = timeNow;
                power += INCREMENT * incDecScl;
            }
            else if(decrPwr)
            {
                double timeNow = bumpTimer.seconds();
                if (timeNow - lastBumpTime < 0.5) incDecScl = 10.0;
                lastBumpTime = timeNow;
                power -= INCREMENT * incDecScl;
            }
            power = Range.clip(power, MAX_REV, MAX_FWD);

            //lScl = 0;
            //rScl = 0;
            if(fwd)        { lScl = lScl == 0?1:0; rScl = rScl == 0?1:0; }
            else if (rev)  { lScl = lScl == 0?-1:0; rScl = rScl == 0?-1:0; }
            else if (rtrn) { lScl = lScl == 0?1:0; rScl = lScl == 0?-1:0; }
            else if (ltrn) { lScl = lScl == 0?-1:0; rScl = lScl == 0?1:0; }

            if(tglAct0) actLst.put(0, !actLst.get(0));
            if(tglAct1) actLst.put(1, !actLst.get(1));
            if(tglAct2) actLst.put(2, !actLst.get(2));
            if(tglAct3) actLst.put(3, !actLst.get(3));

            int sScl = 1;
            if(lstr) sScl = -1;

            for(int m = 0; m < MAX_MOTORS; m++)
            {
                DcMotorEx mot = motors.get(m);
                if(mot == null) continue;
                if(lstr || rstr)
                {
                    if(m == 0 || m == 3) motPwrs.set(m,  sScl * power);
                    else                 motPwrs.set(m, -sScl * power);
                }
                else
                {
                    if (m % 2 == 0) motPwrs.set(m, lScl * power);
                    else            motPwrs.set(m, rScl * power);
                }

                if(mot != null)
                {
                    t1 = timer.milliseconds();
                    encPos = mot.getCurrentPosition();
                    motVel = mot.getVelocity();
                    t2 = timer.milliseconds();
                    if (actLst.get(m))
                    {
                        if (useSpd) mot.setVelocity(MAX_CPS * motPwrs.get(m));
                        else mot.setPower(motPwrs.get(m));

                        t3 = timer.milliseconds();
                        dtg = t2 - t1;
                        dts = t3 - t2;
                        ttg += dtg;
                        tts += dts;

                        if (strtTmr)
                        {
                            ntc = 0;
                            e0 = encPos;
                            tv = 0.0;
                            motVel = 0.0;
                            t0 = timer.milliseconds();
                        }

                        if (tact)
                        {
                            denc = encPos - e0;
                            dtt = (timer.milliseconds() - t0) / 1000.0;
                            cps = (double) denc / dtt;
                            ntc++;
                            tv += motVel;
                            av = tv / ntc;
                            if(motVel > maxV) maxV = motVel;
                        }

                        String rMode;
                        switch (mot.getMode())
                        {
                            case RUN_USING_ENCODER:
                                rMode = "U";
                                break;
                            case RUN_WITHOUT_ENCODER:
                                rMode = "W";
                                break;
                            default:
                                rMode = "X";
                                break;
                        }
                        String dirStr = "U";
                        switch (mot.getDirection())
                        {
                            case FORWARD:
                                dirStr = "F";
                                break;
                            case REVERSE:
                                dirStr = "R";
                                break;
                        }
                        dashboard.displayText(m,
                            String.format(Locale.US,"C:%5d P:%.2f V:%.1f RM:%s D:%s MX:%.1f",
                            encPos, motPwrs.get(m), motVel, rMode, dirStr, maxV));
                    }
                    else
                    {
                        dashboard.displayText(m, "");
                    }
                }
            }
            numCycles++;

            double effKf = 32767/maxV;
            double vcmpKf = effKf * strtV/12.0;
            dashboard.displayText(MAX_MOTORS + p++,
                String.format(Locale.US,"Pwr %4.2f BM:%s SM:%s F:%.2f V:%.1f",
                    power, bcmStr, useSpd, vcmpKf, strtV));

            dashboard.displayText(MAX_MOTORS + p++,
                String.format(Locale.US,"Avg G/S Time %.1f %.1f",
                    ttg / numCycles, tts / numCycles));

            dashboard.displayText(MAX_MOTORS + p++,
                String.format(Locale.US,"T: %.1f C: %d CPS: %.1f AV: %.1f",
                    dtt, denc, cps, av));

            dashboard.displayText(MAX_MOTORS + p++, "Press Stop to end test.");
            dashboard.displayText(MAX_MOTORS + p++, "Decr/Incr pwr : L/R bump");
            dashboard.displayText(MAX_MOTORS + p++, "TglAct 0,1,2,3 : A,B,X,Y");
            dashboard.displayText(MAX_MOTORS + p++, "FWD/Rev : Dpad Up/Down");
            dashboard.displayText(MAX_MOTORS + p++, "L/R Turn : Dpad Left/Right");
            dashboard.displayText(MAX_MOTORS + p++, "Tgl Tmr     : L plunge");
            dashboard.displayText(MAX_MOTORS + p++, "Tgl Dirs    : R plunge");
            dashboard.displayText(MAX_MOTORS + p++, "Tgl Run mode: L trig");
            dashboard.displayText(MAX_MOTORS + p,   "Tgl Spd mode: R trig");

            sleep(CYCLE_MS);
        }

        for(int m = 0; m < MAX_MOTORS; m++)
        {
            DcMotorEx mot = motors.get(m);
            if(mot != null)
            {
                mot.setPower(0.0);
            }
        }

        dashboard.displayText(MAX_MOTORS + 1, "Done." );
    }

    double getBatteryVoltage()
    {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
            {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
