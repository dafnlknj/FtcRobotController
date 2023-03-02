package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.AprilTagBot;
import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Auto (Park)", group="Auto")
public class AutoPark extends LinearOpMode {

    protected AprilTagBot robot = new AprilTagBot(this);

    @Override
    public void runOpMode() {
        int drivingLaneX = -33000;
        int droppingPosX = -22000;
        int stackY = -86500;

        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        int pos = robot.detect(); // 0 = left, 1 = middle, 2 = right

        if (pos == 0) {
            robot.driveToCoordinate(-40000, -3000, 0, 1000, 0.5, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(-40000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        } else if (pos == 1) {
            robot.driveToCoordinate(0, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        } else {
            robot.driveToCoordinate(40000, -3000, 0, 1000, 0.5, false);
            robot.waitForCoordinateDrive();
            robot.driveToCoordinate(40000, -70000, 0, 1000, 0.5, true);
            robot.waitForCoordinateDrive();
        }
    }
}

