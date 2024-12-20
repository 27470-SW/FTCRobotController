package org.firstinspires.ftc.teamcode.field;

import static org.firstinspires.ftc.teamcode.field.Field.FirstLocation.*;
import static org.firstinspires.ftc.teamcode.field.Field.StartPos.*;
import static org.firstinspires.ftc.teamcode.field.Route.Movement.*;
import static org.firstinspires.ftc.teamcode.field.Route.Heading.*;
import static org.firstinspires.ftc.teamcode.field.Route.TeamElement.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Park {
    String TAG = "SJH+DTTB";
    Route route;
    private Field.Parks stackToBack;
    private Field.Parks pixelStack;

    private Route.TeamElement teamElement;
    private Field.Alliance alliance;
    public Park(Route constructorRoute) {
route = constructorRoute;
    }

    public void makeTraj(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.FirstLocation firstLocation, Field.Parks stackToBack) {
        route.addLocation(route.startCSRedHigh, START, HEAD_LINEAR);
        route.addFunction(route::closeClaw);
        route.addEvent (Route.Action.WAIT, 1);
        route.addLocation(route.startCBlueHigh, LINE, HEAD_LINEAR);
      //  route.addLocation(route.parked, LINE, HEAD_LINEAR);
    }

    private void goToBackdrop(Pose2d backdrop){
 /*
        if(stackToBack == WALL)
            viaWall(backdrop);
        else if(stackToBack == CENTER){
            viaCenter(backdrop);
        }
        else{//DOOR

            viaDoor(backdrop);
        }

  */
    }


    private void viaWall(Pose2d backdrop){
        route.addLocation(route.byBlueLoadStation, SPLINE, HEAD_LINEAR, route.zero);
        route.makeNewTraj();
        route.addLocation(route.moveTowardsRedBackdropLft, LINE, HEAD_LINEAR,route.ninety);
        route.addFunction(route::closeClaw, 1.5);
        route.addLocation(backdrop, SPLINE, HEAD_LINEAR,route.ninety);
    }

    private void viaCenter(Pose2d backdrop){
        route.addEvent(Route.Action.TANGENT, route.twoseventy);
        route.addLocation(route.whatchamacallit, SPLINE, HEAD_LINEAR);
        route.addFunction(route::closeClaw, 1.5);
        route.addLocation(backdrop, LINE, HEAD_LINEAR);
    }

    private void viaDoor(Pose2d backdrop){
        route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR, route.zero);
        route.addEvent(Route.Action.TANGENT, route.oneHundred);
        route.addFunction(route::closeClaw, 2);
        if(alliance == Field.Alliance.RED && teamElement == RIGHT){
            route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.ten);
        }else {
            route.addLocation(backdrop, SPLINE, HEAD_LINEAR, route.thirty);
        }
    }


    private void qualifierRoute(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.FirstLocation firstLocation) {
        if (alliance == Field.Alliance.RED) {
            switch ((Field.StartPos) startPos) {
                case START_SPECIMENS:

                    route.addLocation(route.start, SPLINE, HEAD_LINEAR);
                    switch (teamElement) {
                        case LEFT:
                            // Red Left Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedLeftTapeBackdropAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedLeftTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromLRedBackdropTape, LINE, HEAD_LINEAR);
////                            route.addMovement(TURN, -0.9);
//                            route.addEvent(Route.Action.WAIT, 0.3);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.1);
////                            route.addMovement(TURN, 0.7);
//                            route.addEvent(Route.Action.WAIT, 0.3);
//                            route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.1);
////                            route.addMovement(TURN, -0.5);
//                            route.addEvent(Route.Action.WAIT, 0.1);
                            // my old route
                            route.addEvent(Route.Action.TANGENT, route.oneFifteen);
//                            route.addLocation(route.dropPixelRedLeftTapeBackdropAdj,SPLINE,HEAD_LINEAR, 180);
                            route.addLocation(route.dropPixelRedLeftTapeBackdrop, LINE, HEAD_LINEAR, route.twofifity);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.15);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::closeClaw);
                            route.addEvent(Route.Action.WAIT, .2);
                            route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR, Math.toRadians(100));
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .75);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, .2);
                            break;
                        case CENTER:
                            // Red Center Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addLocation(route.dropPixelRedCenterBackTapeAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedCenterTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromCRedBackdropTape, LINE, HEAD_LINEAR);
////                            route.addMovement(TURN, -0.9);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropRedCenterBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
////                            route.addMovement(TURN, -0.5);
//                            route.addEvent(Route.Action.WAIT, 0.2);
                            // my old route
                            route.addLocation(route.dropPixelRedCenterTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos, .3);
                            route.addEvent(Route.Action.WAIT, .3);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                            route.addEvent(Route.Action.TANGENT, route.thirty);
                            route.addFunction(route::closeClaw);
                            route.addLocation(route.dropOnBackdropRedCenterBackdrop, SPLINE, HEAD_LINEAR, route.oneFifteen);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                        case RIGHT:
                            // Red Right Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedRightTapeBackdropAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedRightTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromRRedBackdropTape, LINE, HEAD_LINEAR);
////                            route.addMovement(TURN, -0.9);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropRedRightBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromRedBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
////                            route.addMovement(TURN, -0.5);
//                            route.addEvent(Route.Action.WAIT, 0.2);
                            //my old route
                            route.addLocation(route.dropPixelRedRightTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos, .3);
                            route.addEvent(Route.Action.WAIT, .3);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                        route.addEvent(Route.Action.TANGENT, route.thirty);
                            route.addFunction(route::closeClaw);
                            route.addLocation(route.dropOnBackdropRedRightBackdrop, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .35);

//                        route.addFunction(route::outPixel);
//                        route.addEvent(Route.Action.WAIT, .5);
                            break;
                    }
                    break;
                case START_SAMPLES:
                    route.addLocation(route.start, START, HEAD_LINEAR);
                    switch (teamElement) {
                        case LEFT:
                            // Red Left Stacks (7252)
//                            route.addLocation(route.moveAwayFromWallRedStacks, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.3);
//                            if (firstLocation == BACKDROP) {
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                            } else if (firstLocation == PIXEL_WALL) {
//                                route.addLocation(route.pickUpPixelStackLeft, LINE, HEAD_LINEAR);
//                                route.addFunction(route::armToIntake);
//                                route.addEvent(Route.Action.WAIT, 1);
//                                route.addFunction(route::intakes);
//                                route.addEvent(Route.Action.WAIT, .35);
//                                route.addFunction(route::armDropSpikePos);
//                                route.addFunction(route::outFrontPixel);
//                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addMovement(TURN, .7);
//                            }
//                            route.addLocation(route.byBlueLoadStation, LINE, HEAD_LINEAR);
//                            route.addLocation(route.moveTowardsRedBackdropLft, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.moveTowardsRedBackdropHdAdjLft, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropRedLeftStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDropHigher);
//                            route.addLocation(route.dropOnBackdropRedLeftStacksHi, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.75);
//                            route.addLocation(route.moveTowardsRedBackdropHdAdj, LINE, HEAD_LINEAR);
                            // my old route
                            route.addEvent(Route.Action.TANGENT, route.oneHundred);
                            route.addFunction(route::armDropSpikePos, 1);
                            route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .4);
                            route.addEvent(Route.Action.TANGENT, Math.toRadians(200));
                            if (firstLocation == BACKDROP) {
                                route.makeNewTraj();
                                route.addFunction(route::closeClaw);
                                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR, route.oneeighty);
                                route.addFunction(route::armDropSpikePos);
                                route.addEvent(Route.Action.TANGENT, route.oneHundred);
                                route.addFunction(route::closeClaw, 2);
                                route.addLocation(route.dropOnBackdropRedLeftStacks, SPLINE, HEAD_LINEAR, route.forty);
                            } else if (firstLocation == PIXEL_WALL) {
                                route.addLocation(route.pickUpPixelStackLeft, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
                                route.addEvent(Route.Action.WAIT, .35);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .15);
                                route.addFunction(route::armDropSpikePos);
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addMovement(TURN, .7);
                                route.addEvent(Route.Action.TANGENT, route.onefifity);
                                route.addFunction(route::armDropSpikePos, 1);
                                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR, route.zero);
                                route.addEvent(Route.Action.TANGENT, route.oneHundred);
                                route.addFunction(route::closeClaw, 2);
//            route.addLocation(route.dropCenterPixelBackwards, SPLINE, HEAD_LINEAR,route.zero);
                                route.addLocation(route.dropCenterPixelBackwardsLeftBackdrop, SPLINE, HEAD_LINEAR, route.zero);
//                                MoveToBackdropFromPixelStack t4 = new MoveToBackdropFromPixelStack(route);
//                                t4.makeTraj(stackToBack, pixelStack,LEFT, alliance);
                                route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR);
                            } else if (firstLocation == PIXEL_CENTER) {
                                route.addLocation(route.pickUpPixelStackCenterStacks, LINE, HEAD_LINEAR);
                                route.makeNewTraj();
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
                                route.addEvent(Route.Action.WAIT, .35);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .15);
                                route.addFunction(route::armDropSpikePos);
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addMovement(TURN, .7);
                                route.addEvent(Route.Action.TANGENT, route.onefifity);
                                route.addFunction(route::armDropSpikePos, 1);
                                route.addLocation(route.underdoor, SPLINE, HEAD_LINEAR, route.zero);
                                route.addEvent(Route.Action.TANGENT, route.oneHundred);
                                route.addFunction(route::closeClaw, 2);
//            route.addLocation(route.dropCenterPixelBackwards, SPLINE, HEAD_LINEAR,route.zero);
                                route.addLocation(route.dropCenterPixelBackwardsLeftBackdrop, SPLINE, HEAD_LINEAR, route.zero);
//                                MoveToBackdropFromPixelStack t4 = new MoveToBackdropFromPixelStack(route);
//                                t4.makeTraj(stackToBack, pixelStack,LEFT, alliance);
                                route.addLocation(route.dropOnBackdropRedLeftBackdrop, LINE, HEAD_LINEAR);
                            }
//                            route.addLocation(route.byBlueLoadStation, SPLINE, HEAD_LINEAR, 0);
//                            route.addLocation(route.moveTowardsRedBackdropLft, LINE, HEAD_LINEAR,route.ninety);
//                            route.addFunction(route::armToDrop, .5);
//                            route.addLocation(route.dropOnBackdropRedLeftStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .75);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropRedLeftStacksHi, LINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .75);
                            break;
                        case CENTER:
                            // Red Center Stacks (7252)
//                            route.addLocation(route.moveAwayFromWallRedStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.dropPixelRedCenterStkTapeAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedCenterTapeStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            if (firstLocation == BACKDROP) {
//                                route.addLocation(route.moveFromRedCenterTapeStacks, LINE, HEAD_LINEAR);
//                            } else if (firstLocation == PIXEL_WALL) {
//                                route.addEvent(Route.Action.TANGENT, Math.toRadians(-10));
//                                route.addMovement(TURN, .5);
//                                route.addLocation(route.pickUpPixelStackLeftCenter, SPLINE, HEAD_LINEAR, 270);
//                                route.addFunction(route::armToIntake);
//                                route.addEvent(Route.Action.WAIT, 1);
//                                route.addFunction(route::intakes);
//                                route.addEvent(Route.Action.WAIT, .35);
//                                route.addFunction(route::armDropSpikePos);
//                                route.addFunction(route::outFrontPixel);
//                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addMovement(TURN, -.5);
//                            }
//                            route.addLocation(route.byBlueLoadStation, LINE, HEAD_LINEAR);
//                            route.addLocation(route.backAwayFromRedTape, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveTowardsRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addMovement(TURN, .5);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.moveTowardsRedBackdropHdAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropRedCenterStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDropHigher);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.dropOnBackdropRedCenterStacksHi, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.moveTowardsRedBackdropHdAdj, LINE, HEAD_LINEAR);
                            //my old route
                            route.addLocation(route.dropPixelRedCenterTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, .05);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .4);
                            if (firstLocation == BACKDROP) {

                                goToBackdrop(route.dropOnBackdropRedCenterStacks);
                            } else if (firstLocation == PIXEL_WALL) {
                                route.addEvent(Route.Action.TANGENT, Math.toRadians(-10));
                                //route.addMovement(TURN, .5);
                                route.addLocation(route.pickUpPixelStackRight, SPLINE, HEAD_LINEAR, 270);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, .3);
                                route.addFunction(route::intakes);
                                route.addEvent(Route.Action.WAIT, .35);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .2);
                                //route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
                                //route.addMovement(TURN, -.5);
                                route.addLocation(route.dropOnBackdropRedCenterStacks, SPLINE, HEAD_LINEAR, 270);
                            }
//                            route.addLocation(route.byBlueLoadStation, SPLINE, HEAD_LINEAR);
//                            route.addLocation(route.moveTowardsRedBackdrop, LINE, HEAD_LINEAR,route.ninety);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
//                            route.addFunction(route::armToDrop, .5);
//                            route.addLocation(route.dropOnBackdropRedCenterStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            route.addFunction(route::armToDropMid);
                            route.addLocation(route.dropOnBackdropRedCenterStacksHi, LINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, 1);
                            break;
                        case RIGHT:
                            // Red Right Stacks (7252)
//                            route.addLocation(route.moveAwayFromWallRedStacks, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelRedRightTapeStacksAdj, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addLocation(route.dropPixelRedRightTapeStacks, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            if (firstLocation == BACKDROP) {
//                                route.addLocation(route.moveFromRedRightTapeStacks, LINE, HEAD_LINEAR);
//                            } else if (firstLocation == PIXEL_WALL) {
//                                route.addLocation(route.pickUpPixelStackLeft, LINE, HEAD_LINEAR);
//                                route.addFunction(route::armToIntake);
//                                route.addEvent(Route.Action.WAIT, 1);
//                                route.addFunction(route::intakes);
//                                route.addEvent(Route.Action.WAIT, .35);
//                                route.addFunction(route::armDropSpikePos);
//                                route.addFunction(route::outFrontPixel);
//                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
//                                route.addMovement(TURN, -.5);
//                            }
//                            route.addLocation(route.byBlueLoadStation, LINE, HEAD_LINEAR);
//                            route.addLocation(route.backAwayFromRedTape, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveTowardsRedBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armToDrop);
////                            route.addLocation(route.moveTowardsRedBackdropHdAdjRt,LINE, HEAD_LINEAR);
//                            route.addMovement(TURN, .5);
////                            route.addEvent(Route.Action.WAIT, .3);
//                            route.addLocation(route.dropOnBackdropRedRightStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDropHigher);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.dropOnBackdropRedRightStacksHi, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.moveTowardsRedBackdropHdAdj, LINE, HEAD_LINEAR);
                            //my old route
                            route.addFunction(route::armDropSpikePos, 1.5);
                            route.addLocation(route.dropPixelRedRightTapeStacks, LINE, HEAD_LINEAR);
//            route.addEvent(Route.Action.TANGENT, route.twoTen);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            if (firstLocation == BACKDROP) {
                                goToBackdrop(route.dropOnBackdropRedRightStacks);
                            } else if (firstLocation == PIXEL_WALL) {
                                route.addLocation(route.pickUpPixelStackLeft, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
                                route.addEvent(Route.Action.WAIT, .35);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .15);
                                route.addFunction(route::armDropSpikePos);
                                // route.addLocation(route.moveFromRedLeftTapeStacks, LINE, HEAD_LINEAR);
                                //route.addLocation(route.dropPixelRedLeftTapeStacks, LINE, HEAD_LINEAR);
                                //route.addMovement(TURN, -.5);
                                route.addLocation(route.dropOnBackdropRedRightStacks, LINE, HEAD_LINEAR);
                            }
//                            route.addLocation(route.byBlueLoadStation, SPLINE, HEAD_LINEAR);
//                            route.addLocation(route.moveTowardsRedBackdrop, LINE, HEAD_LINEAR,route.ninety);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.dropOnBackdropRedRightStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropRedRightStacksHi, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                    }
            }
        } else {
            switch ((Field.StartPos) startPos) {
                case START_SPECIMENS:
                    route.addLocation(route.start, SPLINE, HEAD_LINEAR);
                    switch (teamElement) {
                        case LEFT:
                            // Blue Left Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueLeftTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromLBlueBackdropTapeAdj, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromLBlueBackdropTape, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.dropOnBackdropBlueLeftBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);

                            //my old route
                            route.addLocation(route.dropPixelRedRightTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos, 1);
                            route.addEvent(Route.Action.WAIT, .3);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                        route.addEvent(Route.Action.TANGENT, route.thirty);
                            route.addFunction(route::closeClaw);
                            route.addLocation(route.dropOnBackdropBlueLeftBackdrop, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            //red before
//                            route.addLocation(route.dropPixelBlueLeftTapeBackdrop2,LINE,HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, .2);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, .5);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.dropOnBackdropBlueLeftBackdrop, SPLINE, HEAD_LINEAR, Math.toRadians(80));
//                            route.addEvent(Route.Action.TANGENT, 120);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                        case CENTER:
                            // Blue Center Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueCenterBackTapeAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueCenterTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromCBlueBackdropTape, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropBlueCenterBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);

                            // my old route
                            route.addLocation(route.dropPixelRedCenterTapeBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos, .3);
                            route.addEvent(Route.Action.WAIT, .3);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
//                            route.addEvent(Route.Action.TANGENT, route.thirty);
                            route.addFunction(route::closeClaw);
                            route.addLocation(route.dropOnBackdropBlueCenterBackdrop, SPLINE, HEAD_LINEAR, route.oneFifteen);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            //red before
//                            route.addLocation(route.dropPixelBlueCenterTapeBackdrop,LINE,HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, .3);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, .5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.TANGENT, 180);
//                            route.addLocation(route.dropOnBackdropBlueCenterBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                        case RIGHT:
                            // Blue Right Backdrop (7252)
//                            route.addLocation(route.moveAwayFromWallBlueBackdrop, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueRightTapeBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDrop);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveAwayFromRBlueBackdropTape, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropBlueRightBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromBlueBackdropBk, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
                            //my old route
                            route.addEvent(Route.Action.TANGENT, route.oneFifteen);
//                            route.addLocation(route.dropPixelRedLeftTapeBackdropAdj,SPLINE,HEAD_LINEAR, 180);
                            route.addLocation(route.dropPixelRedLeftTapeBackdrop, LINE, HEAD_LINEAR, route.twofifity);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, 0.2);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .6);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
                            route.addFunction(route::closeClaw);
                            route.addEvent(Route.Action.WAIT, .2);
                            route.addLocation(route.dropOnBackdropBlueRightBackdrop, LINE, HEAD_LINEAR);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .75);
                            // red before
//                            route.addLocation(route.moveAwayFromWallBlueBackdrop,SPLINE,HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueRightTapeBackdrop,SPLINE,HEAD_LINEAR,route.twoTen);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, .3);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, .6);
//                            route.addFunction(route::armToDrop, 3);
//                            route.addLocation(route.dropOnBackdropBlueRightBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                    }
                    break;
                case START_SAMPLES:
                    route.addLocation(route.start, START, HEAD_LINEAR);
                    switch (teamElement) {
                        case LEFT:
                            // Blue Left Stacks (7252)
//                            route.addLocation(route.moveAwayFromWallBlueStacks, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueRightTapeStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.dropPixelBlueLeftTapeStacks, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            if (firstLocation == BACKDROP) {
//                                route.addLocation(route.moveFromBlueLeftTapeStacks, LINE, HEAD_LINEAR);
//                            } else if (firstLocation == PIXEL_WALL) {
//                                route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR);
//                                route.addFunction(route::armToIntake);
//                                route.addEvent(Route.Action.WAIT, 1);
//                                route.addFunction(route::intakes);
//                                route.addEvent(Route.Action.WAIT, .35);
//                                route.addFunction(route::armDropSpikePos);
//                                route.addFunction(route::outFrontPixel);
//                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
//                            }
//                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.moveTowardsBlueBackdropHdAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropBlueLeftStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDropHigher);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.dropOnBackdropBlueLeftStacksHi, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.reverseFromBlueBackdropStk, LINE, HEAD_LINEAR);
                            // my old route
//                            route.addLocation(route.dropPixelBlueRightTapeStacks,LINE,HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos, 1.50);
                            route.addLocation(route.dropPixelBlueLeftTapeStacks, LINE, HEAD_LINEAR);
//            route.addEvent(Route.Action.TANGENT, route.twoTen);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            if (firstLocation == BACKDROP) {
                                goToBackdrop(route.dropOnBackdropBlueLeftStacks);
                            } else if (firstLocation == PIXEL_WALL) {
                                route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
                                route.addEvent(Route.Action.WAIT, .35);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .15);
                                route.addFunction(route::armDropSpikePos);
                            }
//                            route.addLocation(route.backAwayFromBlueTape, SPLINE, HEAD_LINEAR);
//                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR,route.ninety);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.dropOnBackdropBlueLeftStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            route.addFunction(route::armToDropHigher);
                            route.addLocation(route.dropOnBackdropBlueLeftStacksHi, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                        case CENTER:
                            // Blue Center Stacks (7252)
//                            route.addLocation(route.moveAwayFromWallBlueStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.dropPixelBlueCenterStkTapeAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueCenterTapeStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            if (firstLocation == BACKDROP) {
//                                route.addLocation(route.moveFromBlueCenterTapeStacks, LINE, HEAD_LINEAR);
//                            } else if (firstLocation == PIXEL_WALL) {
//                                route.addMovement(TURN, -.5);
//                                route.addLocation(route.pickUpPixelStackRight, SPLINE, HEAD_LINEAR, Math.toRadians(270));
//                                route.addFunction(route::armToIntake);
//                                route.addEvent(Route.Action.WAIT, 1);
//                                route.addFunction(route::intakes);
//                                route.addEvent(Route.Action.WAIT, .45);
//                                route.addFunction(route::armDropSpikePos);
//                                route.addFunction(route::outFrontPixel);
//                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
//                            }
//                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.moveTowardsBlueBackdropHdAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropBlueCenterStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDropHigher);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.dropOnBackdropBlueCenterStacksHi, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPixel);
//                            route.addLocation(route.backupFromBackdropBlueStacksCenter, LINE, HEAD_LINEAR);
                            //my old route
                            route.addLocation(route.dropPixelBlueCenterTapeStacks, LINE, HEAD_LINEAR);
                            route.addFunction(route::armDropSpikePos);
                            route.addEvent(Route.Action.WAIT, .2);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .4);
                            if (firstLocation == BACKDROP) {
                                goToBackdrop(route.dropOnBackdropBlueCenterStacks);
                            } else if (firstLocation == PIXEL_WALL) {
                                // route.addMovement(TURN, -.5);
                                route.addLocation(route.pickUpPixelStackRight, SPLINE, HEAD_LINEAR, Math.toRadians(270));
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
                                route.addEvent(Route.Action.WAIT, .35);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .15);
                                route.addFunction(route::armDropSpikePos);
                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
                                route.addEvent(Route.Action.TANGENT, 180);

                            }
//                            route.addLocation(route.backAwayFromBlueTape, SPLINE, HEAD_LINEAR);
//                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR,route.ninety);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.dropOnBackdropBlueCenterStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            route.addFunction(route::closeClaw);
                            route.addLocation(route.dropOnBackdropBlueCenterStacksHi, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                        case RIGHT:
                            // Blue Right Stacks (7252)
//                            route.addLocation(route.moveAwayFromWallBlueStacks, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropPixelBlueRightTapeStacks, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armDropSpikePos);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::outPurplePixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            if (firstLocation == BACKDROP) {
//                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
//                            } else if (firstLocation == PIXEL_WALL) {
//                                route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR);
//                                route.addFunction(route::armToIntake);
//                                route.addEvent(Route.Action.WAIT, 1);
//                                route.addFunction(route::intakes);
//                                route.addEvent(Route.Action.WAIT, .35);
//                                route.addFunction(route::armDropSpikePos);
//                                route.addFunction(route::outFrontPixel);
//                                route.addEvent(Route.Action.WAIT, .2);
//                                route.addLocation(route.moveFromBlueRightTapeStacks, LINE, HEAD_LINEAR);
//                            }
//                            route.addLocation(route.backAwayFromBlueTape, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.moveTowardsBlueBackdropHdAdj, LINE, HEAD_LINEAR);
//                            route.addLocation(route.dropOnBackdropBlueRightStacks, LINE, HEAD_LINEAR);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addFunction(route::armToDropHigher);
//                            route.addEvent(Route.Action.WAIT, 0.5);
//                            route.addLocation(route.dropOnBackdropBlueRightStacksHi, LINE, HEAD_LINEAR);
//                            route.addEvent(Route.Action.WAIT, 0.2);
//                            route.addFunction(route::outPixel);
//                            route.addEvent(Route.Action.WAIT, 0.3);
//                            route.addLocation(route.reverseFromBlueBackdropStk, LINE, HEAD_LINEAR);
                            // my old route
                            route.addFunction(route::armDropSpikePos, 1.25);
                            route.addLocation(route.dropPixelBlueRightTapeStacks, LINE, HEAD_LINEAR);
                            route.addEvent(Route.Action.TANGENT, route.twoTen);
                            route.addFunction(route::outPurplePixel);
                            route.addEvent(Route.Action.WAIT, .55);
                            route.addFunction(route::closeClaw);
                            if (firstLocation == BACKDROP) {
                                route.addFunction(route::armDropSpikePos, 1.5);
                                goToBackdrop(route.dropOnBackdropBlueRightStacks);
                            } else if (firstLocation == PIXEL_WALL) {
                                route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR);
                                route.addFunction(route::armToIntake);
                                route.addEvent(Route.Action.WAIT, 1);
                                route.addFunction(route::intakes);
                                route.addEvent(Route.Action.WAIT, .35);
                                route.addFunction(route::outFrontPixel);
                                route.addEvent(Route.Action.WAIT, .15);
                                route.addFunction(route::armDropSpikePos);
                                route.addEvent(Route.Action.WAIT, .2);
                            }
//                            route.addLocation(route.backAwayFromBlueTape, SPLINE, HEAD_LINEAR, Math.toRadians(125));
//                            route.addEvent(Route.Action.TANGENT, 180);
//                            route.addLocation(route.moveTowardsBlueBackdrop, LINE, HEAD_LINEAR,route.ninety);
//                            route.addEvent(Route.Action.TANGENT, route.ninety);
//                            route.addFunction(route::armToDrop);
//                            route.addLocation(route.dropOnBackdropBlueRightStacks, SPLINE, HEAD_LINEAR,route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            route.addFunction(route::armToDropMid);
                            route.addLocation(route.dropOnBackdropBlueRightStacksHi, SPLINE, HEAD_LINEAR, route.ninety);
                            route.addFunction(route::outPixel);
                            route.addEvent(Route.Action.WAIT, .5);
                            break;
                    }
            }


        }

        if ((alliance == Field.Alliance.BLUE && startPos == START_SPECIMENS)){
            route.addLocation(route.dropCenterPixelBackwards, LINE, HEAD_LINEAR);
        }
    }

    private void moveStartToLeft(){

    }




    protected void startPos(){

    }

    protected void firstMovement(Route.TeamElement teamElement, Field.Alliance alliance, PositionOption startPos, Field.FirstLocation firstLocation)
    {
        if((teamElement ==LEFT &&alliance== Field.Alliance.RED ||teamElement ==RIGHT &&alliance== Field.Alliance.BLUE)  && startPos == START_SPECIMENS && firstLocation == BACKDROP){
            route.addLocation(route.startCSRedHigh,START,HEAD_DEFAULT);
//            route.addEvent(Route.Action.TANGENT,route.ninetyFive);
            route.addLocation(route.extraPointHigh,SPLINE,HEAD_LINEAR);
            route.addLocation(route.teamElementLeft,SPLINE,HEAD_LINEAR);
            route.addFunction(route::armDropSpikePos, 1.5);
            route.addEvent(Route.Action.WAIT, .3);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .6);
            route.addEvent(Route.Action.TANGENT, route.ninety);
            route.addFunction(route::closeClaw, .5);
            route.addLocation(route.dropLeftPixel, SPLINE, HEAD_LINEAR,route.oneFifteen);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .5);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .5);
        }
        if((teamElement ==RIGHT && alliance== Field.Alliance.RED || teamElement ==LEFT && alliance== Field.Alliance.BLUE) && startPos == START_SPECIMENS && firstLocation == BACKDROP){
            route.addLocation(route.start,START,HEAD_DEFAULT);
            route.addLocation(route.teamElementRight,LINE,HEAD_LINEAR);
            route.addFunction(route::armDropSpikePos, .3);
            route.addEvent(Route.Action.WAIT, .3);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .6);
            route.addEvent(Route.Action.TANGENT, route.thirty);
            route.addFunction(route::closeClaw);
            route.addLocation(route.dropRightPixel, SPLINE, HEAD_LINEAR,route.oneFifteen);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .5);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .5);
        }
        if(teamElement == Route.TeamElement.CENTER && startPos == START_SPECIMENS && firstLocation == BACKDROP){
            route.addLocation(route.startCSRedHigh,START,HEAD_DEFAULT);
            route.addLocation(route.teamElementCenter,LINE,HEAD_LINEAR);
            route.addFunction(route::armDropSpikePos, .3);
            route.addEvent(Route.Action.WAIT, .3);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .6);
            route.addEvent(Route.Action.TANGENT, route.thirty);
            route.addFunction(route::closeClaw);
            route.addLocation(route.dropCenterPixel, SPLINE, HEAD_LINEAR,route.oneFifteen);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .5);
            route.addFunction(route::outPixel);
            route.addEvent(Route.Action.WAIT, .5);
        }
        if((teamElement ==LEFT && alliance== Field.Alliance.RED ||teamElement ==RIGHT && alliance== Field.Alliance.BLUE)  && startPos == START_SAMPLES && firstLocation == BACKDROP){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addFunction(route::armDropSpikePos, .3);
            route.addEvent(Route.Action.TANGENT, route.oneHundred);
            route.addLocation(route.teamElementLeftLow,LINE,HEAD_LINEAR);
            route.addFunction(route::outPixel );
            route.addEvent(Route.Action.WAIT,.5);
            route.addLocation(route.underRigging, SPLINE, HEAD_LINEAR);
            route.addLocation(route.aboveRigging, LINE, HEAD_LINEAR,route.ninety);
            route.addEvent(Route.Action.TANGENT, route.ninety);
            route.addFunction(route::closeClaw, .5);
            route.addLocation(route.dropLeftPixel, SPLINE, HEAD_LINEAR,route.ninety);
            route.addFunction(route::outPixel );
            route.addEvent(Route.Action.WAIT,.5);
        }
        if((teamElement ==RIGHT && alliance== Field.Alliance.RED || teamElement==LEFT && alliance== Field.Alliance.BLUE) && startPos == START_SAMPLES && firstLocation == BACKDROP){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addFunction(route::armDropSpikePos, 1.5);
            route.addLocation(route.extraPointLow,LINE,HEAD_LINEAR);
//            route.addEvent(Route.Action.TANGENT, route.twoTen);
            route.addFunction(route::outPixel );
            route.addEvent(Route.Action.WAIT,.5);
            route.addLocation(route.underRigging, SPLINE, HEAD_LINEAR);
            route.addLocation(route.aboveRigging, LINE, HEAD_LINEAR,route.ninety);
            route.addEvent(Route.Action.TANGENT, route.ninety);
            route.addFunction(route::closeClaw, .5);
            route.addLocation(route.dropRightPixel, SPLINE, HEAD_LINEAR,route.ninety);
            route.addFunction(route::outPixel );
            route.addEvent(Route.Action.WAIT,.5);
        }
        if(teamElement == Route.TeamElement.CENTER && startPos == START_SAMPLES && firstLocation == BACKDROP){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addFunction(route::armDropSpikePos, 1);
            route.addLocation(route.teamElementCenterLow,LINE,HEAD_LINEAR);
            route.addFunction(route::outPixel );
            route.addEvent(Route.Action.WAIT,.5);
            route.addLocation(route.underRigging, SPLINE, HEAD_LINEAR);
            route.addLocation(route.aboveRigging, LINE, HEAD_LINEAR,route.ninety);
            route.addEvent(Route.Action.TANGENT, route.ninety);
            route.addFunction(route::closeClaw, .5);
            route.addLocation(route.dropCenterPixel, SPLINE, HEAD_LINEAR,route.ninety);
            route.addFunction(route::outPixel );
            route.addEvent(Route.Action.WAIT,.5);
        }
        if(teamElement ==LEFT && startPos == START_SAMPLES && firstLocation == PIXEL_WALL){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(330));
            route.addLocation(route.teamElementLeftLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackRight, SPLINE, HEAD_LINEAR,Math.toRadians(270));
        }
        if(teamElement ==LEFT && startPos == START_SAMPLES && firstLocation == PIXEL_CENTER){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(330));
            route.addLocation(route.teamElementLeftLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackRight, SPLINE, HEAD_LINEAR,Math.toRadians(270));
            route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR,Math.toRadians(200));
        }
        if(teamElement ==LEFT && startPos == START_SAMPLES && firstLocation == PIXEL_DOOR){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(330));
            route.addLocation(route.teamElementLeftLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackRight, SPLINE, HEAD_LINEAR,Math.toRadians(270));
            route.addLocation(route.pickUpPixelStackLeft, LINE, HEAD_LINEAR,Math.toRadians(330));
        }
        if(teamElement ==RIGHT && startPos == START_SAMPLES && firstLocation == PIXEL_WALL){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addLocation(route.teamElementRightLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR,Math.toRadians(130));
        }
        if(teamElement ==RIGHT && startPos == START_SAMPLES && firstLocation == PIXEL_CENTER){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(220));
            route.addLocation(route.teamElementRightLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackCenter, LINE, HEAD_LINEAR,Math.toRadians(200));
        }
        if(teamElement ==RIGHT && startPos == START_SAMPLES && firstLocation == PIXEL_DOOR){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(220));
            route.addLocation(route.teamElementRightLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackLeft, LINE, HEAD_LINEAR,Math.toRadians(200));
        }
        if(teamElement == Route.TeamElement.CENTER && startPos == START_SAMPLES && firstLocation == PIXEL_CENTER){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
            route.addLocation(route.teamElementCenterLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackCenter, SPLINE, HEAD_LINEAR,Math.toRadians(200));
        }
        if(teamElement == Route.TeamElement.CENTER && startPos == START_SAMPLES && firstLocation == PIXEL_DOOR){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(300));
            route.addLocation(route.teamElementCenterLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackLeft, SPLINE, HEAD_LINEAR,Math.toRadians(200));
        }
        if(teamElement == Route.TeamElement.CENTER && startPos == START_SAMPLES && firstLocation == PIXEL_WALL){
            route.addLocation(route.startCSRedLow,START,HEAD_DEFAULT);
            route.addEvent(Route.Action.TANGENT, Math.toRadians(330));
            route.addLocation(route.teamElementCenterLow,LINE,HEAD_LINEAR);
            route.addLocation(route.pickUpPixelStackRight, LINE, HEAD_LINEAR,Math.toRadians(270));
        }
        route.printLastPose();
//        if(!(startPos == BACKDROP)){
//            MoveToBackdropFromPixelStack t1 = new MoveToBackdropFromPixelStack(route);
//            if(firstLocation == PIXEL_WALL)
//                t1.makeTraj(Field.Highways.DOOR, Field.Highways.WALL);
//            else if(firstLocation == PIXEL_CENTER)
//                t1.makeTraj(Field.Highways.DOOR, Field.Highways.CENTER);
//            else //DOOR
//                t1.makeTraj(Field.Highways.DOOR, Field.Highways.DOOR);
//        }
    }



}


