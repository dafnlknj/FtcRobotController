package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Auto (Left)", group="Auto")
public class AutoLeft extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        int drivingLaneX = -33000;
        int droppingPosX = -20000;
        int stackY = -87000;

        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        robot.driveToCoordinate(drivingLaneX, -3000, 0, 1000, 0.5, false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(drivingLaneX, stackY, -90, 1000, 0.5, false);
        robot.waitForCoordinateDrive();
        robot.sleep(500);
        robot.driveToCoordinate(droppingPosX, stackY, -90, 700, 0.15, true);
        robot.waitForCoordinateDrive();
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.LOADING_DONE);
        robot.turretSet = -398;
        robot.loadingStateTrigger = true;

        robot.autoScoring(2000, 0.38, true);
        robot.autoScoring(1400, 0.32, true);
        robot.autoScoring(750, 0.26, true);
        robot.autoScoring(750, 0.22, true);

        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(800);
        robot.scoreCone(true, false);
        robot.sleep(800);

//        robot.flipperStackHeight = 0.41;
//        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
//        robot.sleep(3000);
//        robot.driveToCoordinate(-37000, -86000, -90, 750, 0.2);
//        robot.waitForCoordinateDrive();
//        robot.grabCone(true);
//        robot.sleep(1000);
//        robot.driveToCoordinate(-25000, -86000, -90, 750, 0.2);
//        robot.waitForCoordinateDrive();
//        robot.waitForState(FSMBot.ConeState.LOADING_DONE);
//        robot.loadingStateTrigger = true;
//        robot.waitForState(FSMBot.ConeState.SCORING);
//        robot.sleep(1000);
//        robot.scoreCone(true, false);
//        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
        robot.coneState = FSMBot.ConeState.DRIVING;
        if (pos == 0) {
            robot.driveToCoordinate(drivingLaneX, stackY, -90, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
            robot.sleep(500);
        } else if (pos == 1) {
            robot.driveToCoordinate(10000, stackY, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(10000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
            robot.sleep(500);
        } else {
            robot.driveToCoordinate(45000, stackY, -90, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
            robot.sleep(500);
        }
    }
}

