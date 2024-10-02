package org.firstinspires.ftc.teamcode.field;



import org.firstinspires.ftc.teamcode.util.PreferenceMgr;

public class TimeForPixelRun {
    ITD_Route route;
    int count;

    private int MAX_LOOPS = 3;
    public TimeForPixelRun(Route constructorRoute, Field.Highways pixelStack) {
//        super(constructorRoute.robot, constructorRoute.teamElement, constructorRoute.startPos, constructorRoute.parkPos, constructorRoute.alliance, constructorRoute.firstLocation, constructorRoute.highways, constructorRoute.pixelStacks);

        int temp = PreferenceMgr.getCircuit();
        if (temp != -1){
            MAX_LOOPS = temp;
        }
        route = (ITD_Route)constructorRoute;
        count = 0;

//        // next four lines need to be the same as in PPlayRoute currently line 302
//        MoveToPixelStackFromBackdrop t3 = new MoveToPixelStackFromBackdrop(this);
//        t3.makeTraj(Field.Highways.WALL, pixelStack);
//        //addFunction(route::pickUpPixels);
//        MoveToBackdropFromPixelStack t4 = new MoveToBackdropFromPixelStack(this);
//        t4.makeTraj(Field.Highways.DOOR, pixelStack);
//        //addFunction(route::dropPixels);

//        finalizeTrajSeq();
    }

   public boolean isTime(double time){
        if (time + route.totalDur <= 30){
            count++;
            if(count < MAX_LOOPS)
                return true;
            else
                return false;
        }
        else{
            return false;
        }
   }

   /* public Boolean isTime(){
        if(count < 2)
        {
            count++;
            return true;
        }
        return false;
    }*/




}


