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
        //constants, for ease of programming
        int drivingLaneX = -33000;
        int droppingPosX = -22000;
        int stackY = -84500;
        int turretRot = -400;

        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        //signal sleeve detection
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        //drive forward and avoid junctions
        robot.driveToCoordinate(18000, -30000, 0, 20000, 1, false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(5000, stackY+5000, 0, 7000, 1, false);
        robot.waitForCoordinateDrive();
        //dropping position
        robot.driveToCoordinate(droppingPosX, stackY, -90, 700, 1.5, 0.25, true);
        robot.waitForCoordinateDrive();
        //realign angle using gyro sensor
        robot.reAngle(0);
        //dropping position
        robot.driveToCoordinate(droppingPosX, stackY, -90, 500, 1, 0.1, true);
        robot.waitForCoordinateDrive();
        //setup FSM and start extension
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.EXTENDING_STAGE_2);
        robot.turretSet = turretRot; //405
        robot.loadingStateTrigger = true;
        //parameters: adjustable distance to wall, maximum driving power, intake height, turret rotation, left side?, last one?
        robot.autoScoringNoDist(4500, 0.3, 0.39, turretRot, true, false); //0.37
        robot.autoScoringNoDist(4000, 0.3, 0.32, turretRot, true, false); //0.32
        robot.autoScoringNoDist(2000, 0.3, 0.26, turretRot, true, false); //0.26
        robot.autoScoringNoDist(1500, 0.3, 0.21, turretRot, true, false);
        robot.autoScoringNoDist(2000, 0.3, 0.16, turretRot, true, true);
        //manually finishes scoring, because I made the repeating method stupidly
        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(500);
        robot.scoreCone(true, false);
        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
        //raises claw for ease of driving
        robot.coneState = FSMBot.ConeState.DRIVING;
        //drive to correct parking
        if (pos == 0) {
            robot.driveToCoordinate(-29000, stackY, 0, 3000, 1, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-29000, -70000, 0, 1000, 1, true);
            robot.waitForCoordinateDrive();
        } else if (pos == 1) {
            robot.driveToCoordinate(10000, stackY, 0, 3000, 1, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(10000, -70000, 0, 1000, 1, true);
            robot.waitForCoordinateDrive();
        } else {
            robot.driveToCoordinate(50000, stackY, -90, 7000, 1, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(50000, -70000, 0, 1000, 1, true);
            robot.waitForCoordinateDrive();
        }
    }
}

