package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Auto (Right 5 Cone)", group="Auto")
public class AutoFiveRight extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        int drivingLaneX = -33000;
        int droppingPosX = 20000;
        int stackY = -85600;

        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        robot.driveToCoordinate(-18000, -30000, 0, 20000, 1, false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(-5000, stackY+5000, 0, 7000, 1, false);
        robot.waitForCoordinateDrive();
        //robot.sleep(500);
        robot.driveToCoordinate(droppingPosX, stackY, 90, 700, 1.5, 0.25, true);
        robot.waitForCoordinateDrive();
        robot.reAngle(-2);
        robot.driveToCoordinate(droppingPosX, stackY, 90, 500, 1, 0.1, true);
        robot.waitForCoordinateDrive();
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.EXTENDING_STAGE_2);
        robot.turretSet = 405; //405
        robot.loadingStateTrigger = true;

        robot.autoScoringNoDist(6600, 0.3, 0.39, false, false); //0.37
        robot.autoScoringNoDist(6000, 0.3, 0.32, false, false); //0.32
        robot.autoScoringNoDist(4200, 0.3, 0.26, false, false); //0.26
        robot.autoScoringNoDist(3500, 0.3, 0.21, false, false);
        robot.autoScoringNoDist(3800, 0.3, 0.16, false, true);

        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(500);
        robot.scoreCone(true, false);
        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
        //robot.sleep(900);
        robot.coneState = FSMBot.ConeState.DRIVING;
        if (pos == 0) {
            robot.driveToCoordinate(-50000, stackY, 90, 7000, 1, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-50000, -70000, 0, 1000, 1, true);
            robot.waitForCoordinateDrive();
        } else if (pos == 1) {
            robot.driveToCoordinate(-10000, stackY, 0, 3000, 1, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-10000, -70000, 0, 1000, 1, true);
            robot.waitForCoordinateDrive();
        } else {
            robot.driveToCoordinate(29000, stackY, 0, 3000, 1, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(29000, -70000, 0, 1000, 1, true);
            robot.waitForCoordinateDrive();
        }
    }
}

