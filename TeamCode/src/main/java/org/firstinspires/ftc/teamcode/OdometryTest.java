package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.OdometryBot;

@Autonomous(name="Odometry Test", group="Tests")

public class OdometryTest extends LinearOpMode {

    protected OdometryBot robot = new OdometryBot(this);

    @Override
    public void runOpMode() {
        int drivingLaneX = -33000;
        int droppingPosX = -19000;
        int stackY = -84500;
        robot.init(hardwareMap);
        waitForStart();
        robot.driveToCoordinate(18000, -30000, 0, 13000, 1, false);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(5000, stackY, 0, 7000, 1, false);
        robot.waitForCoordinateDrive();
        //robot.sleep(500);
        robot.driveToCoordinate(droppingPosX, stackY, -90, 700, 0.25, true);
        robot.waitForCoordinateDrive();
        robot.driveToCoordinate(droppingPosX, stackY, -90, 700, 1, 0.1, true);
        robot.waitForCoordinateDrive();
    }
}
