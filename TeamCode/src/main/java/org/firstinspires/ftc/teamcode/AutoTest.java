package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Auto Test", group="Tests")
public class AutoTest extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect();
        robot.driveToCoordinate(12500, -5000, 0, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(12500, -16000, 0, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.sleep(1000);
        robot.driveToCoordinate(12500, -9000, 0, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(32000, -9000, 0, 750, 0.2);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(32000, -51000, 0, 750, 0.15);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        if (pos == 0) {

        } else if (pos == 1) {
            robot.driveToCoordinate(-6000, -51000, 0, 750, 0.15);
            robot.waitForCoordinateDrive();
            robot.sleep(200);
        } else {
            robot.driveToCoordinate(-45000, -50000, 0, 750, 0.15);
            robot.waitForCoordinateDrive();
            robot.sleep(200);
        }
    }
}

