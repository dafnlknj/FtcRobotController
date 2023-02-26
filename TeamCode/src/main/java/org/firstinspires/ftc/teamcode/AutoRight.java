package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Auto (Right)", group="Auto")
public class AutoRight extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        int drivingLaneX = 36000;
        int droppingPosX = 21000;
        int stackY = -85600;

        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        robot.driveToCoordinate(drivingLaneX, -3000, 0, 1000, 0.5, false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(drivingLaneX, stackY, 90, 1000, 0.5, false);
        robot.waitForCoordinateDrive();
        //robot.sleep(500);
        robot.driveToCoordinate(droppingPosX, stackY, 90, 700, 0.15, true);
        robot.waitForCoordinateDrive();
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.LOADING_DONE);
        robot.turretSet = 360;
        robot.loadingStateTrigger = true;

        robot.autoScoring(8.1, 0.1, 0.38, false);
        robot.autoScoring(7.8, 0.1, 0.32, false);
        robot.autoScoring(7.8, 0.1, 0.26, false);
        robot.autoScoring(7.8, 0.1, 0.22, false);
        //robot.autoScoring(8.3, 0.2, 0.16, true);


        robot.waitForState(FSMBot.ConeState.SCORING);
        //robot.sleep(800);
        robot.scoreCone(true, false);
        robot.sleep(900);
        robot.coneState = FSMBot.ConeState.DRIVING;
        if (pos == 0) {
            robot.driveToCoordinate(-45000, stackY, 90, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-45000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        } else if (pos == 1) {
            robot.driveToCoordinate(-10000, stackY, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-10000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        } else {
            robot.driveToCoordinate(29000, stackY, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(29000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        }
    }
}

