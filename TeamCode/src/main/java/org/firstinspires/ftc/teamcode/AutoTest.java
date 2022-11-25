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
        robot.driveToCoordinate(11000, -5000, 0, 750, 1);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(11000, -16000, 0, 750, 1);
        robot.waitForCoordinateDrive();
        robot.sleep(1000);
        robot.driveToCoordinate(11000, -9000, 0, 750, 1);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(31000, -9000, 0, 750, 1);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(31000, -41000, 0, 750, 1);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        if (pos == 0) {

        } else if (pos == 1) {
            robot.driveToCoordinate(-7000, -41000, 0, 750, 1);
            robot.waitForCoordinateDrive();
            robot.sleep(200);
        } else {
            robot.driveToCoordinate(-46000, -40000, 0, 750, 1);
            robot.waitForCoordinateDrive();
            robot.sleep(200);
        }
    }
}

