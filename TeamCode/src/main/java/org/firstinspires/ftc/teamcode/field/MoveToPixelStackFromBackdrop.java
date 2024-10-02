package org.firstinspires.ftc.teamcode.field;



import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Field.Highways.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class MoveToPixelStackFromBackdrop {
    Route route;
    public MoveToPixelStackFromBackdrop(Route constructorRoute) {
route = constructorRoute;
    }

    public void makeTraj(Field.Highways highway, Field.Highways pixelStack, Field.Alliance alliance, Route.TeamElement teamElement, PositionOption startPos) {
        if((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.LEFT && startPos == Field.StartPos.START_STACKS))
        {
            route.addLocation(route.dropLeftPixelBackwardsBlue, LINE, HEAD_LINEAR); //make new point
        } else if (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_STACKS){
            route.addLocation(route.dropRightPixelBackwardsRed, LINE, HEAD_LINEAR); //make new point
        } else if((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_STACKS))
        {
            route.addLocation(route.dropRightPixelBackwardsBlue, LINE, HEAD_LINEAR); //make new point
        }else if (alliance == Field.Alliance.RED && teamElement == Route.TeamElement.LEFT && startPos == Field.StartPos.START_STACKS){
            route.addLocation(route.dropLeftPixelBackwardsRed, LINE, HEAD_LINEAR); //make new point
        }else if((alliance == Field.Alliance.BLUE && teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_STACKS) )
        {
            route.addLocation(route.dropCenterPixelBackwardsBlue, LINE, HEAD_LINEAR); //make new point
        }else if(alliance == Field.Alliance.RED && teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_STACKS) {
            route.addLocation(route.dropCenterPixelBackwardsRed, LINE, HEAD_LINEAR);
        } else if(alliance == Field.Alliance.RED && teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_BACKDROP){
                route.addLocation(route.dropCenterPixelBackwardsRedBd, LINE, HEAD_LINEAR);
        }else if(alliance == Field.Alliance.RED && teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_BACKDROP) {
            route.addLocation(route.dropRightPixelBackwardsRedBd, LINE, HEAD_LINEAR);
        } else if(alliance == Field.Alliance.RED && teamElement == Route.TeamElement.LEFT && startPos == Field.StartPos.START_BACKDROP) {
            route.addLocation(route.dropLeftPixelBackwardsRedBd, LINE, HEAD_LINEAR);

        }else {
            route.addLocation(route.dropCenterPixelBackwards, LINE, HEAD_LINEAR);
        }
        if (alliance == Field.Alliance.RED) {


            if (highway == WALL && pixelStack == WALL) {
                RobotLog.dd(RobotLog.TAG, "got where i thought!");
                route.printLastPose();

                if(teamElement == Route.TeamElement.LEFT) {
                    route.addEvent(Route.Action.TANGENT, route.threetwenty);
                    route.addFunction(route::armDropSpikePos, 1.5);
                    route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT, Math.toRadians(240));
                    route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                    //try 1
                    route.addFunction(route::armToIntake);
                    route.addFunction(route::intakes);
                    route.addLocation(route.pickUpPixelStackRightRedLeft, SPLINE, HEAD_LINEAR);
                    //
                }else if(teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_STACKS){
                    route.addEvent(Route.Action.TANGENT, route.threetwenty);
                    route.addFunction(route::armDrivePos, 1.5);
                    route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT);
                    route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                    //try 1
                    route.addFunction(route::armToIntake);
                    route.addFunction(route::intakes);
                    route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
                    //
                }else{
                    route.addEvent(Route.Action.TANGENT, route.threefortyfive);
                    route.addFunction(route::armDrivePos, 1.5);
                    route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT);
                    route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                    //try 1
                    route.addFunction(route::armToIntake);
                    route.addFunction(route::intakes);
                    route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
                }

            }

            if (highway == WALL && pixelStack == CENTER) {

               // route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, route.threefortyfive);
                route.addFunction(route::armDropSpikePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
                route.addEvent(Route.Action.TANGENT, route.twotwenty);
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR, route.twosixty);
                route.addEvent(Route.Action.TANGENT, route.oneten);
            }
            if (highway == WALL && pixelStack == DOOR) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT,route.threefortyfive);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR);
            }
            if (highway == CENTER && pixelStack == WALL) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == CENTER && pixelStack == CENTER) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(270));
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == CENTER && pixelStack == DOOR) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(180));
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == DOOR && pixelStack == WALL) {
                RobotLog.dd(RobotLog.TAG, "DOOR/WALL");

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(210));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == DOOR && pixelStack == CENTER) {
                RobotLog.dd(RobotLog.TAG, "DOOR/CENTER");

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(210));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR);
            }
            if (highway == DOOR && pixelStack == DOOR) {
                RobotLog.dd(RobotLog.TAG, "DOOR/DOOR");

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(210));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR);
            }

//            route.addFunction(route::armToIntake);
//            route.addEvent(Route.Action.WAIT, .3);
//            route.addFunction(route::intakes);
//            route.addEvent(Route.Action.WAIT, .5);
//            route.addFunction(route::outFrontPixel);
        }else if (alliance == Field.Alliance.BLUE) {
            if (highway == WALL && pixelStack == WALL) {
                RobotLog.dd(RobotLog.TAG, "got where i thought!");
                route.printLastPose();
//                route.addLocation(route.dropCenterPixel, LINE, HEAD_LINEAR);
                route.addEvent(Route.Action.TANGENT,route.threetwenty);
                route.addFunction(route::armDropSpikePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT,route.sixty);
                route.addEvent(Route.Action.TANGENT, route.twotwentyfive);
                //try 1
                route.addFunction(route::armToIntake);
                route.addFunction(route::intakes);
                //
                if(teamElement == Route.TeamElement.RIGHT && startPos == Field.StartPos.START_BACKDROP){
                    route.addLocation(route.pickUpPixelStackRightCircuitBlueRight, SPLINE, HEAD_LINEAR);
                }else if(teamElement == Route.TeamElement.CENTER && startPos == Field.StartPos.START_STACKS){
                    route.addLocation(route.pickUpPixelStackRightCircuitBlueCenter, SPLINE, HEAD_LINEAR);
                }else{
                route.addLocation(route.pickUpPixelStackRightCircuitBlue, SPLINE, HEAD_LINEAR);
            }

            }

            if (highway == WALL && pixelStack == CENTER) {

                // route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, route.threefortyfive);
                route.addFunction(route::armDropSpikePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR, Math.toRadians(-45));
                route.addEvent(Route.Action.TANGENT, route.twotwenty);
                route.addLocation(route.pickUpPixelStackCenterBlue, SPLINE, HEAD_LINEAR, route.twosixty);
                route.addEvent(Route.Action.TANGENT, route.oneten);
            }
            if (highway == WALL && pixelStack == DOOR) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT,route.threefortyfive);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR, Math.toRadians(-45));
                route.addLocation(route.pickUpPixelStackLeftBlue, SPLINE, HEAD_LINEAR);
            }
//            if (highway == WALL && pixelStack == WALL) {
//                RobotLog.dd(RobotLog.TAG, "got where i thought!")
//                route.printLastPose();
////                route.addLocation(route.dropCenterPixel, LINE, HEAD_LINEAR);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(200));
//                route.addFunction(route::armDrivePos, 1.5);
//                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_CONSTANT);
//                route.addLocation(route.pickUpPixelStackRightCircuit, LINE, HEAD_LINEAR);
//
//            }
//
//            if (highway == WALL && pixelStack == CENTER) {
//
////                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(200));
//                route.addFunction(route::armDrivePos, 1.5);
//                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(220));
//                route.addLocation(route.pickUpPixelStackCenter, LINE, HEAD_LINEAR, Math.toRadians(30));
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(110));
//            }
//            if (highway == WALL && pixelStack == DOOR) {
//
////                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
//                route.addEvent(Route.Action.TANGENT, Math.toRadians(200));
//                route.addFunction(route::armDrivePos, 1.5);
//                route.addLocation(route.pointForRobotToPixelStack, SPLINE, HEAD_LINEAR);
//                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR);
//            }
            if (highway == CENTER && pixelStack == WALL) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == CENTER && pixelStack == CENTER) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(270));
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == CENTER && pixelStack == DOOR) {

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underMiddlePoint, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(180));
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR, Math.toRadians(200));
            }
            if (highway == DOOR && pixelStack == WALL) {
                RobotLog.dd(RobotLog.TAG, "DOOR/WALL");

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackRightCircuit, SPLINE, HEAD_LINEAR);
            }
            if (highway == DOOR && pixelStack == CENTER) {
                RobotLog.dd(RobotLog.TAG, "DOOR/CENTER");

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR);
            }
            if (highway == DOOR && pixelStack == DOOR) {
                RobotLog.dd(RobotLog.TAG, "DOOR/DOOR");

//                route.addLocation(route.dropCenterPixel, LINE, HEAD_SPLINE);
                route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
                route.addFunction(route::armDrivePos, 1.5);
                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR);
                route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR);
            }

        }
        Pose2d backedUp = new Pose2d(route.getLastPose().getX(), route.getLastPose().getY()+1.2, route.getLastPose().getHeading());
        route.addLocation(backedUp, LINE, HEAD_LINEAR);
        route.makeNewTraj();
        route.addFunction(route::outFrontPixel);
    }

}
//thisisourogpickup
// route.addFunction(route::armToIntake);
//         route.addEvent(Route.Action.WAIT, .3);
//         route.addFunction(route::intakes);
//         route.addEvent(Route.Action.WAIT, .3);
//         Pose2d backedUp = new Pose2d(route.getLastPose().getX(), route.getLastPose().getY()+1, route.getLastPose().getHeading());
//         route.addLocation(backedUp, LINE, HEAD_LINEAR);
//         route.addEvent(Route.Action.WAIT, .3);
//         route.addFunction(route::outFrontPixel);

//Try 1
//Pose2d backedUp = new Pose2d(route.getLastPose().getX(), route.getLastPose().getY()+1, route.getLastPose().getHeading());
//        route.addLocation(backedUp, LINE, HEAD_LINEAR);
//                route.addFunction(route::outFrontPixel);