package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.FSMBot;

@Autonomous(name="Drive Test", group="Tests")
public class DriveTest extends LinearOpMode {

    protected FSMBot robot = new FSMBot(this);

    @Override
    public void runOpMode() {
        robot.isAuto = true;
        robot.init(hardwareMap);
        waitForStart();
        robot.sleep(2000);
        robot.readyToGrab = true;
        robot.waitForState(FSMBot.ConeState.LOADING_DONE);
        robot.turretSet = -390;
        robot.loadingStateTrigger = true;
        robot.autoScoring(7.5, 0.2, 0.38, true);
        robot.sleep(1000);
        robot.waitForState(FSMBot.ConeState.SCORING);
        robot.sleep(500);
        robot.scoreCone(true, false);
        robot.sleep(1000);


//        int distanceFromStart = Math.abs(robot.horizontal.getCurrentPosition());
//        int drivingDistance = distanceFromStart + 27000;
//        robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, CameraBot.autoSide.BLUE, drivingDistance, 500, 0);
//        robot.drivingDone = true;
//        robot.setDropHeight(0);
//        robot.autoGrabFreight(0.2, CameraBot.autoSide.BLUE);

        //robot.driveAgainstWallWithEncodersVertical(robot.DIRECTION_FORWARD, robot.SIDE_BLUE, 40000, 500, 200);
    }
}
