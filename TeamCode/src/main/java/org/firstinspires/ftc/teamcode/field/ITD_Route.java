package org.firstinspires.ftc.teamcode.field;

import static android.os.SystemClock.sleep;

import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.HalDashboard;
import org.firstinspires.ftc.teamcode.opModes.ITD_Auton;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import static org.firstinspires.ftc.teamcode.image.ITD_Detector.Position.*;
import static org.firstinspires.ftc.teamcode.field.Field.StartPos.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

public class ITD_Route extends Route
{
  private static final String TAG = "SJH_PPR";

  public ITD_Route(       MecanumBot robot,
                          PositionOption startPos,
						  Field.Parks parkPos,
                          Field.FirstLocation firstLocation)
  {
    super(robot, startPos, parkPos, firstLocation);
  }


  protected void goPark()
  {

  }

  /*
  Use the builder functions in Route to build routes in initTrajectories2
  common examples are:
        addLocation(Pose2d, Movement, Heading) - to move the robot to that location
        addFunction(functionName) - to perform a function
        addEvent(WAIT, waitTime) - to add a wait period
        addParkPosition(ParkPos, Pose2d, Movement, Heading) - to define where to park and how to move there
  */
    public void dropPixel(){
        //TODO: add code to make this work
    }

    public void dropPixels(){
        outPixel();
        sleep(500);
//        armToDropHigher();
//        runToDistSensor();
        outPixel();
        sleep(500);
    }
    public void runToDistSensor(){

    }

    public void pickUpPixel(){
        //TODO: add code to make this work
    }

    public void pickUpPixels(){
        pickUpPixel();
        pickUpPixel();
    }



   protected void initTrajectories2()
   {
       RobotLog.dd(TAG, "in InitTrajectories2");
       dashboardConfig();

       Pose2d lastPose;


       RobotLog.dd(TAG, "making route");
	   
       if(startPos == START_SAMPLES) {
           SampleRoute t1 = new SampleRoute(this);
           t1.makeTraj(startPos, parkPos, firstLocation);

       }
       else if(startPos == START_SPECIMENS)
       {
           SpecimenRoute t1 = new SpecimenRoute(this);
           t1.makeTraj(startPos, parkPos, firstLocation);

       }
       lastPose = this.getEnd();
	   
	   RobotLog.dd(TAG, "route made");

/*
       RobotLog.dd(TAG, "making park");
       try{
           Pose2d parkStrt;
            if(PreferenceMgr.getCircuit() == 0){
                parkStrt = new Pose2d(dropOnTapeThenBackdrop.getLastPose().getX(), dropOnTapeThenBackdrop.getLastPose().getY(), dropOnTapeThenBackdrop.getLastPose().getHeading());
            }else{
                parkStrt = new Pose2d(circuit1.getLastPose().getX(), circuit1.getLastPose().getY(), circuit1.getLastPose().getHeading());
            }

           moveToPark.addLocation(parkStrt, START, HEAD_CONSTANT);
       }catch(Exception e){ RobotLog.ww(TAG,"unable to add lastPose(2)");}
       MoveToPark t5 = new MoveToPark(moveToPark);
       t5.makeTraj(parkPos, alliance);
       RobotLog.dd(TAG, "park made");

       createExtraMovementRoutes();
*/
        //Always do this at the end of initTrajectories2
       // finalizeAllTrajSeqs();
	   finalizeTrajSeq();
    }

/*
    public void initExtraMovements(ITD_Auton opMode) {
        if ((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.LEFT && startPos == Field.StartPos.START_SAMPLES)  || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_SAMPLES)) {
            //do nothing; the adjustment got baked in
        }else if (startPos == Field.StartPos.START_SPECIMENS&& teamElement == TeamElement.RIGHT && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdRight);
        } else if ((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.RIGHT) || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.LEFT))
        {
            //do nothing; the adjustment got baked in
        }else if((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_SAMPLES)  || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.CENTER))
        {
            //do nothing; the adjustment got baked in
        }
        else if (startPos == Field.StartPos.START_SAMPLES && alliance == Field.Alliance.RED){
            opMode.doAuton(extraMovementRedStacksLeft);
        }
        else if(startPos == Field.StartPos.START_SAMPLES){
            opMode.doAuton(extraMovement2);
        }else if (startPos == Field.StartPos.START_SPECIMENS && teamElement == TeamElement.LEFT && alliance == Field.Alliance.RED){
//            opMode.doAuton(extraMovementRedBdLeft);
        }else if (startPos == Field.StartPos.START_SPECIMENS&& (teamElement == TeamElement.CENTER) && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdCenter);
        }else if (startPos == Field.StartPos.START_SPECIMENS&& teamElement == TeamElement.LEFT && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdLeft);
        }

    }
*/
    public void runRoute(ITD_Auton opMode){

        opMode.doAuton(this);
      //  initExtraMovements(opMode);

//        outPixel();
//        sleep(500);

        //TODO: try to re-add TimeForPixelRun to change what happens based on current time
//        TimeForPixelRun q1 = new TimeForPixelRun(timeForPixelRun, pixelStacks[0]);
//        double time =0;
////        while(q1.isTime(time)) { // if the contents of this loop changes umm TimeForPixelRun needs to also change!

  //      if(PreferenceMgr.getCircuit() >=1) { opMode.doAuton(circuit1); dropPixels(); armToDropHigher(); }
    //    if(PreferenceMgr.getCircuit() >=2) { opMode.doAuton(circuit2); dropPixels();}
      //  if(PreferenceMgr.getCircuit() >=3) { opMode.doAuton(circuit3);}


            // time = find total duration of current route
//            time = calcTimeOrSomething();
//        }

      //  opMode.doAuton(moveToPark);
    }


    protected void dashboardConfig(){		//TODO: Fix for ITD
        try {

            HalDashboard dashboard = CommonUtil.getInstance().getDashboard();
            int lnum = 8;
            dashboard.displayText(lnum++, "Start:    " + startPos);
            dashboard.displayText(lnum++, "Park Position:  " + parkPos);
            dashboard.displayText(lnum++, "First Location:  " + firstLocation);
            //dashboard.displayText(lnum++, "Curcuit:    " + curcuit);
            dashboard.displayText(lnum++, "Curcuit 1 " + highways[0] + "," + pixelStacks[0] + "," + highways[1]);
            dashboard.displayText(lnum++, "Curcuit 2 " + highways[2] + "," + pixelStacks[1] + "," + highways[3]);
            dashboard.displayText(lnum++, "Curcuit 3 " + highways[4] + "," + pixelStacks[2] + "," + highways[5]);
        }
        catch (Exception e){}
    }

}
