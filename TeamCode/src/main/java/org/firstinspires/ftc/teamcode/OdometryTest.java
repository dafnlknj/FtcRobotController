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
        robot.init(hardwareMap);
        waitForStart();
        robot.driveToCoordinate(29000, -5000, 0, 750, 0.5);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(29000, -18000, 0, 750, 0.5);
        robot.waitForCoordinateDrive();
        robot.sleep(1000);
        robot.driveToCoordinate(29000, -9000, 0, 750, 0.5);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(49000, -9000, 0, 750, 0.5);
        robot.waitForCoordinateDrive();
        robot.sleep(200);
        robot.driveToCoordinate(49000, -53000, 0, 750, 0.5);
        robot.waitForCoordinateDrive();
        robot.sleep(1000);
        robot.driveToCoordinate(9000, -53000, 0, 750, 0.5);
        robot.waitForCoordinateDrive();
        robot.sleep(1000);
        robot.driveToCoordinate(-31000, -53000, 0, 750, 0.5);
        robot.waitForCoordinateDrive();
        robot.sleep(1000);
    }
}
