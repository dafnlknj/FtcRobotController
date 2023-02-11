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
        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        robot.driveToCoordinate(-40000, -3000, 0, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(-40000, -86000, -90, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(-25000, -86000, -90, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.LOADING_DONE);
        robot.turretSet = -430;
        robot.loadingStateTrigger = true;
        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(1000);
        robot.scoreCone(true, false);
        robot.flipperStackHeight = 0.41;
        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
        robot.sleep(3000);
        robot.driveToCoordinate(-37000, -86000, -90, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.grabCone(true);
        robot.sleep(1000);
        robot.driveToCoordinate(-25000, -86000, -90, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.waitForState(FSMBot.ConeState.LOADING_DONE);
        robot.loadingStateTrigger = true;
        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(1000);
        robot.scoreCone(true, false);
        robot.waitForState(FSMBot.ConeState.GRAB_CONE);
        robot.sleep(5000);
        if (pos == 0) {

        } else if (pos == 1) {

        } else {

        }
    }
}

