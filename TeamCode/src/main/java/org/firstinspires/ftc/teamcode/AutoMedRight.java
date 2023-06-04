package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.GyroHolder;

@Autonomous(name="Auto (Right Med Cone)", group="Auto")
public class AutoMedRight extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        //constants, for ease of programming
        int drivingLaneX = -33000;
        int droppingPosX = 13500;
        int stackY = -85600;
        int turretRot = -450;

        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        //signal sleeve detection
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        robot.driveToCoordinate(-10000, -20000, 0, 10000, 1, false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(-5000, stackY+5000, 0, 7000, 1, false);
        robot.waitForCoordinateDrive();
        //dropping position
        robot.driveToCoordinate(droppingPosX, -83000, 90, 700, 1.5, 0.25, true);
        robot.waitForCoordinateDrive();
        //realign angle using gyro sensor
        robot.reAngle(0);
        //dropping position
        robot.driveToCoordinate(droppingPosX, -83000, 90, 500, 0.5, 0.1, true);
        robot.waitForCoordinateDrive();
        //setup FSM and start extension
        robot.heightIndex = 1;
        robot.setDropHeight();
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.EXTENDING_STAGE_2);
        robot.turretSet = turretRot; //405
        robot.loadingStateTrigger = true;
        //parameters: adjustable distance to wall, dropping pos X, dropping pos Y, height, maximum driving power, intake height, turret rotation, left side?, last one?
        robot.autoScoringNoDist(11000, 10000, -83000, 1, 0.2, 0.39, turretRot, false, false); //0.37
        robot.autoScoringNoDist(10000, 10000, -83000, 1, 0.2, 0.32, turretRot, false, false); //0.32
        robot.autoScoringNoDist(8000, 10000, -83000, 1, 0.2, 0.26, turretRot, false, false); //0.26
        robot.autoScoringNoDist(7500, 10000, -83000, 1, 0.2, 0.21, turretRot, false, false);
        //manually finishes scoring, because I made the repeating method stupidly
        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(500);
        robot.scoreCone(true, false);
        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
        //raises claw for ease of driving
        robot.coneState = FSMBot.ConeState.DRIVING;
        //drive to correct parking
        if (pos == 0) {
            robot.driveToCoordinate(-50000, stackY, 0, 7000, 1, false);
            robot.waitForCoordinateDrive();
//            robot.driveToCoordinate(-50000, -70000, 0, 1000, 1, true);
//            robot.waitForCoordinateDrive();
        } else if (pos == 1) {
            robot.driveToCoordinate(-5000, stackY, 0, 3000, 1, false);
            robot.waitForCoordinateDrive();
//            robot.driveToCoordinate(-10000, -70000, 0, 1000, 1, true);
//            robot.waitForCoordinateDrive();
        } else {
            robot.driveToCoordinate(34000, stackY, 0, 3000, 1, false);
            robot.waitForCoordinateDrive();
//            robot.driveToCoordinate(29000, -70000, 0, 1000, 1, true);
//            robot.waitForCoordinateDrive();
        }
        while (opModeIsActive()) {
            GyroHolder.setHeading(robot.getAngle());
            telemetry.addData("angle:", robot.getAngle());
            telemetry.update();
        }
    }
}

