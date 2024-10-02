package org.firstinspires.ftc.teamcode.field;

import static android.os.SystemClock.sleep;

import org.firstinspires.ftc.teamcode.robot.MecanumBot;
import org.firstinspires.ftc.teamcode.util.CommonUtil;
import org.firstinspires.ftc.teamcode.util.HalDashboard;
import org.firstinspires.ftc.teamcode.opModes.ITD_Auton;
import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

import static org.firstinspires.ftc.teamcode.image.ITD_Detector.Position.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

public class ITD_Route extends Route
{
  private static final String TAG = "SJH_PPR";

  public ITD_Route(MecanumBot robot,
                   TeamElement teamElement,
                   PositionOption startPos,
                   Field.Highways parkPos,
                   Field.Alliance alliance,
                   Field.FirstLocation firstLocation,
                   Field.Highways stackToBack,
                   Field.Highways[] highways,
                   Field.Highways[] pixelStacks)
  {
    super(robot, teamElement, startPos, parkPos, alliance, firstLocation, stackToBack, highways, pixelStacks);
  }


  protected void goPark()
  {
      addParkPosition(LEFT, parkLEFT, LINE, HEAD_LINEAR);
      addParkPosition(RIGHT, parkRIGHT, LINE, HEAD_LINEAR);
      addParkPosition(CENTER, parkCENTER, LINE, HEAD_LINEAR);
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

    protected Route dropOnTapeThenBackdrop;
    protected Route timeForPixelRun;
    protected Route circuit1;
    protected Route circuit2;
    protected Route circuit3;
    protected Route moveToPark;
    protected Route extraMovement;
    protected Route extraMovement2;
    protected Route extraMovementRedBdLeft;
    protected Route extraMovementBlueBdCenter;
    protected Route extraMovementBlueBdLeft;
    protected Route extraMovementBlueBdRight;
    protected Route extraMovementRedStacksLeft;

    protected void initializeRoutes(){
        dropOnTapeThenBackdrop = new Route(this);
        timeForPixelRun = new Route(this);
        circuit1 = new Route(this);
        circuit2 = new Route(this);
        circuit3 = new Route(this);
        moveToPark = new Route(this);
        extraMovement = new Route(this);
        extraMovement2 = new Route(this);
        extraMovementRedBdLeft = new Route(this);
        extraMovementBlueBdCenter = new Route(this);
        extraMovementBlueBdLeft = new Route(this);
        extraMovementBlueBdRight = new Route(this);
        extraMovementRedStacksLeft = new Route(this);
    }

    protected void finalizeAllTrajSeqs(){
        timeForPixelRun.finalizeTrajSeq();
        circuit1.finalizeTrajSeq();
        circuit2.finalizeTrajSeq();
        circuit3.finalizeTrajSeq();
        moveToPark.finalizeTrajSeq();
        extraMovement.finalizeTrajSeq();
        extraMovement2.finalizeTrajSeq();
        extraMovementRedBdLeft.finalizeTrajSeq();
        extraMovementBlueBdCenter.finalizeTrajSeq();
        extraMovementBlueBdLeft.finalizeTrajSeq();
        extraMovementBlueBdRight.finalizeTrajSeq();
        extraMovementRedStacksLeft.finalizeTrajSeq();
        finalizeTrajSeq();
    }

    protected void createExtraMovementRoutes(){
        extraMovement.addLocation(dropCenterPixelAdj, START, HEAD_CONSTANT);
        extraMovement.addLocation(dropCenterPixelBackwards, LINE, HEAD_LINEAR);

        extraMovement2.addLocation(dropCenterPixelStacksAdj, START, HEAD_CONSTANT);
        extraMovement2.addLocation(dropCenterPixelBackwards, LINE, HEAD_LINEAR);

        extraMovementRedBdLeft.addLocation(dropCenterPixelLeftAdj, START, HEAD_CONSTANT);
        extraMovementRedBdLeft.addLocation(dropCenterPixelBackwards, LINE, HEAD_LINEAR);

        extraMovementBlueBdCenter.addLocation(dropCenterPixelCenterAdjBlue, START, HEAD_CONSTANT);
        extraMovementBlueBdCenter.addLocation(dropCenterPixelBackwards, LINE, HEAD_LINEAR);

        extraMovementBlueBdLeft.addLocation(dropCenterPixelLeftAdjBlue, START, HEAD_CONSTANT);
        extraMovementBlueBdLeft.addLocation(dropCenterPixelBackwards, LINE, HEAD_LINEAR);

        extraMovementBlueBdRight.addLocation(dropCenterPixelRightAdjBlue, START, HEAD_CONSTANT);
        extraMovementBlueBdRight.addLocation(dropCenterPixelBackwards, LINE, HEAD_LINEAR);

        extraMovementRedStacksLeft.addLocation(dropCenterPixelLeftAdjRed, START, HEAD_CONSTANT);
        extraMovementRedStacksLeft.addLocation(dropCenterPixelBackwards, LINE, HEAD_LINEAR);
    }

   protected void initTrajectories2()
   {
       RobotLog.dd(TAG, "in InitTrajectories2");
       dashboardConfig();
       initializeRoutes();

       Pose2d lastPose;


       RobotLog.dd(TAG, "making dropOnTapeThenBackdrop");
       DropOnTapeThenBackdrop t1 = new DropOnTapeThenBackdrop(dropOnTapeThenBackdrop);
       t1.makeTraj(teamElement, alliance, startPos, firstLocation, stackHighway);
       dropOnTapeThenBackdrop.finalizeTrajSeq();
       lastPose = dropOnTapeThenBackdrop.getEnd();
       RobotLog.dd(TAG, "dropOnTapeThenBackdrop made");

       int highwayIndex = 0;
       int stackIndex = 0;

       Route circuit = null;
       for(int i=0; i < PreferenceMgr.getCircuit(); i++) {
           switch (i){
               case 2:
                   circuit = circuit3;
                   break;
               case 1:
                   circuit = circuit2;
                   break;
               case 0:
               default:
                   circuit = circuit1;
                   break;
           }
           RobotLog.dd(TAG, "making circuit");

//           try{circuit.addLocation(lastPose, START, HEAD_CONSTANT);}catch(Exception e){ RobotLog.ww(TAG, "unable to add lastPose(1)");}
           MoveToPixelStackFromBackdrop t3 = new MoveToPixelStackFromBackdrop(circuit);
           //TODO: in the next line of code, "teamElement" is only correct for the first circuit. If we do a second circuit, we need to add code here to make this work
           t3.makeTraj(highways[highwayIndex++], pixelStacks[stackIndex],alliance, teamElement, startPos);  //do not increment pixelStack as we want to move to and leave the same stack
TeamElement dropLocation = TeamElement.RIGHT;
if(teamElement== TeamElement.RIGHT){
     dropLocation = TeamElement.LEFT;
}
if(teamElement == TeamElement.LEFT && alliance == Field.Alliance.RED && startPos == Field.StartPos.START_BACKDROP)   //wierd hack; ideally we would change this
{
    dropLocation = TeamElement.CENTER;
}
           MoveToBackdropFromPixelStack t4 = new MoveToBackdropFromPixelStack(circuit);
           t4.makeTraj(highways[highwayIndex++], pixelStacks[stackIndex++], dropLocation, alliance);
           RobotLog.dd(TAG, "circuit made");
           lastPose = circuit.getEnd();
       }

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

        //Always do this at the end of initTrajectories2
       finalizeAllTrajSeqs();
    }

    public void initExtraMovements(ITD_Auton opMode) {
        if ((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.LEFT && startPos == Field.StartPos.START_STACKS)  || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_STACKS)) {
            //do nothing; the adjustment got baked in
        }else if (startPos == Field.StartPos.START_BACKDROP&& teamElement == TeamElement.RIGHT && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdRight);
        } else if ((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.RIGHT) || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.LEFT))
        {
            //do nothing; the adjustment got baked in
        }else if((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_STACKS)  || (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.CENTER))
        {
            //do nothing; the adjustment got baked in
        }
        else if (startPos == Field.StartPos.START_STACKS && alliance == Field.Alliance.RED){
            opMode.doAuton(extraMovementRedStacksLeft);
        }
        else if(startPos == Field.StartPos.START_STACKS){
            opMode.doAuton(extraMovement2);
        }else if (startPos == Field.StartPos.START_BACKDROP && teamElement == TeamElement.LEFT && alliance == Field.Alliance.RED){
//            opMode.doAuton(extraMovementRedBdLeft);
        }else if (startPos == Field.StartPos.START_BACKDROP&& (teamElement == TeamElement.CENTER) && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdCenter);
        }else if (startPos == Field.StartPos.START_BACKDROP&& teamElement == TeamElement.LEFT && alliance == Field.Alliance.BLUE){
            opMode.doAuton(extraMovementBlueBdLeft);
        }

    }

    public void runRoute(ITD_Auton opMode){

        opMode.doAuton(dropOnTapeThenBackdrop);
        initExtraMovements(opMode);

//        outPixel();
//        sleep(500);

        //TODO: try to re-add TimeForPixelRun to change what happens based on current time
//        TimeForPixelRun q1 = new TimeForPixelRun(timeForPixelRun, pixelStacks[0]);
//        double time =0;
////        while(q1.isTime(time)) { // if the contents of this loop changes umm TimeForPixelRun needs to also change!

        if(PreferenceMgr.getCircuit() >=1) { opMode.doAuton(circuit1); dropPixels(); armToDropHigher(); }
        if(PreferenceMgr.getCircuit() >=2) { opMode.doAuton(circuit2); dropPixels();}
        if(PreferenceMgr.getCircuit() >=3) { opMode.doAuton(circuit3);}


            // time = find total duration of current route
//            time = calcTimeOrSomething();
//        }

        opMode.doAuton(moveToPark);
    }

    protected void dashboardConfig(){
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
