package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Auto (Left 5 Cone)", group="Auto")
public class AutoFiveLeft extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        int drivingLaneX = -33000;
        int droppingPosX = -19000;
        int stackY = -84500;

        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        robot.driveToCoordinate(18000, -30000, 0, 13000, 1, false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(5000, stackY, 0, 7000, 1, false);
        robot.waitForCoordinateDrive();
        //robot.sleep(500);
        robot.driveToCoordinate(droppingPosX, stackY, -90, 700, 0.25, true);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(droppingPosX, stackY, -90, 700, 1, 0.1, true);
        robot.waitForCoordinateDrive();
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.LOADING_DONE);
        robot.turretSet = -415; //405
        robot.loadingStateTrigger = true;

        robot.autoScoring(7.7 - 0.9, 0.1, 0.4, true); //0.37
        robot.autoScoring(7.8 - 0.9, 0.1, 0.34, true); //0.32
        robot.autoScoring(7.8 - 0.9, 0.1, 0.28, true); //0.26
        robot.autoScoring(7.8 - 0.9, 0.1, 0.21, true);
        robot.autoScoring(7.4 - 0.9, 0.1, 0.14, true);

        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(500);
        robot.scoreCone(true, false);
        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
        //robot.sleep(900);
        robot.coneState = FSMBot.ConeState.DRIVING;
        if (pos == 0) {
            robot.driveToCoordinate(-29000, stackY, 0, 3000, 0.75, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-29000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        } else if (pos == 1) {
            robot.driveToCoordinate(10000, stackY, 0, 3000, 0.75, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(10000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        } else {
            robot.driveToCoordinate(50000, stackY, -90, 1000, 0.75, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(50000, -70000, 0, 3000, 0.5, true);
            robot.waitForCoordinateDrive();
        }
    }
}

