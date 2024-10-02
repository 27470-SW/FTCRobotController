package org.firstinspires.ftc.teamcode.field;



import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Field.Highways.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Locale;

public class MoveToPark {
    Route route;
    public MoveToPark(Route constructorRoute) {
        route = constructorRoute;
   }

    public void makeTraj(Field.Highways parkPos, Field.Alliance alliance) {

        //route.addLocation(route.dropCenterPixelBackwards,LINE,HEAD_LINEAR);
        if(parkPos == WALL){
//            route.addMovement(TURN, 0.5);
            route.addEvent(Route.Action.TANGENT, route.threetwenty);
            route.addLocation(route.parkWall,SPLINE,HEAD_LINEAR,route.eightyfive);
            route.addFunction(route::armDropSpikePos);
        }
        if(parkPos == DOOR){
            route.addEvent(Route.Action.TANGENT, route.threeFifty);
            route.addLocation(route.parkDoor,SPLINE,HEAD_LINEAR, route.sixtyfive);
            route.addFunction(route::armDropSpikePos);
        }
        else{

        }
        if(parkPos != CENTER){
                Pose2d backedUp = new Pose2d(route.getLastPose().getX(), route.getLastPose().getY()+16, route.getLastPose().getHeading());
                route.addLocation(backedUp, LINE, HEAD_LINEAR);
        }
    }


    protected void startPos(){

    }



}


