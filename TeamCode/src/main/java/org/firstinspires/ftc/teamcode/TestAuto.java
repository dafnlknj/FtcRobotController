package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.firstinspires.ftc.teamcode.bots.SnarmBot;

@Autonomous(name="Test Robot", group="Tests")

public class TestAuto extends LinearOpMode {

    protected OdometryBot robot = new OdometryBot(this); //replace FourWheelDriveBot with whichever Bot is required

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
//        robot.driveToCoordinate(0, 0, 90, 500, 0.15);
//        robot.waitForCoordinateDrive();
        robot.turnInPlace(90, 500, 0.15);
        robot.waitForTurnInPlace();

        //robot.driveToCoordinate(0, -40000, 90, 500, 0.15);
//        robot.goToAngle(90, 0.15);
//        robot.sleep(5000);
//        robot.goToAngle(0, 0.15);
        robot.sleep(15000);
    }
}
