
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.FSMBot;
import org.firstinspires.ftc.teamcode.bots.FlipperBot;
import org.firstinspires.ftc.teamcode.bots.FourWheelDriveBot;
import org.firstinspires.ftc.teamcode.bots.LEDBot;
import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.firstinspires.ftc.teamcode.bots.TurretBot;

@TeleOp(name = "Manual Drive (Demo)")
public class ManualDriveDemo extends LinearOpMode {

    //ElapsedTime runtime = new ElapsedTime();    // Use to determine when end game is starting.

    private FSMBot robot = new FSMBot(this);
    //int count = 0;

    @Override
    public void runOpMode() {

        robot.isAuto = false;

        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

// driving code
            robot.driveByHandFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_button, gamepad2.left_stick_x, gamepad2.left_stick_y, 0, gamepad2.left_stick_button);
            //robot.driveByHandConeCentric(gamepad2.right_stick_x);

            robot.togglePinch(gamepad1.x);
            robot.controlScorer(gamepad2.left_trigger, gamepad2.right_trigger);
            robot.toggleGrabber(gamepad1.b);

            robot.controlExtender(gamepad1.left_trigger, gamepad1.right_trigger);
            robot.controlTurret(gamepad1.dpad_left, gamepad1.dpad_right);
            robot.controlTurret(gamepad2.dpad_left, gamepad2.dpad_right);

            robot.controlFlipper(gamepad2.left_bumper, gamepad2.right_bumper);
            //robot.controlAngle(gamepad2.dpad_up, gamepad2.dpad_down);
            robot.selectDropHeight(gamepad1.dpad_up, gamepad1.dpad_down, gamepad2.dpad_up, gamepad2.dpad_down);
            robot.grabCone(gamepad1.left_bumper);
            robot.scoreCone(gamepad1.right_bumper, false);
            robot.readyToGrab(gamepad1.a, gamepad2.a);
            robot.grabberUp(gamepad1.x, gamepad2.x);
            robot.resetTurretZero(gamepad2.b);
            //robot.setNewLoadingExtension(gamepad2.y);
            robot.resetExtension(gamepad2.y);

            //robot.stopExtender(gamepad2.left_bumper);

            robot.scorerUp(0, gamepad2.left_trigger);

            robot.resetAngle(gamepad1.right_stick_button);
            robot.resetAngle(gamepad2.right_stick_button);

            //robot.opMode.telemetry.addData("time:", runtime.milliseconds());
            robot.opMode.telemetry.update();
            //runtime.reset();

            robot.onLoop(15, "manual drive");
        }
        robot.close();
    }
}