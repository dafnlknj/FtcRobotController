package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;

@Autonomous(name="Auto (Right)", group="Auto")
public class AutoRight extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right
        robot.driveToCoordinate(0, 61000, 0, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.readyToGrab = true;
        robot.sleep(3000);
        robot.turretSet = -480;
        robot.loadingStateTrigger = true;
        robot.sleep(6000);
        robot.scoreCone(true, false);
        robot.sleep(3000);

        if (pos == 0) {
            robot.driveToCoordinate(0, 45000, 0, 750, 0.2);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-40000, 45000, 0, 750, 0.2);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-40000, 65000, 0, 750, 0.2);
            robot.waitForCoordinateDrive();
            robot.sleep(200);

        } else if (pos == 1) {

        } else {
            robot.driveToCoordinate(0, 45000, 0, 750, 0.2);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(40000, 45000, 0, 750, 0.2);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(40000, 65000, 0, 750, 0.2);
            robot.waitForCoordinateDrive();
            robot.sleep(200);
        }
    }
}

