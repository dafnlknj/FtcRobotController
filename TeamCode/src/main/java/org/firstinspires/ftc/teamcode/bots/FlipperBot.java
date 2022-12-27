package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class FlipperBot extends OdometryBot {
    public Servo flipper = null; //
    public Servo flipAngle = null; // range: 0.38-0.85

    public FlipperBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        flipper = hwMap.get(Servo.class, "flipper");
        flipper.setPosition(0.3);
        flipAngle = hwMap.get(Servo.class, "flipAngle");
        flipAngle.setPosition(0.6);
    }

    public void controlFlipper(boolean up, boolean down) {
        if (up) {
            flipper.setPosition(flipper.getPosition()+0.01);
        } else if (down && flipper.getPosition() > 0.13) {
            flipper.setPosition(flipper.getPosition()-0.01);
        }
        if (flipper.getPosition() < 0.5) {
            flipAngle.setPosition(Math.min(flipper.getPosition() * 1.96 + 0.161, 0.85));
        } else {
            flipAngle.setPosition(0.55);
        }
    }

    public void controlAngle(boolean up, boolean down) {
        if (up) {
            flipAngle.setPosition(flipAngle.getPosition()+0.01);
        } else if (down) {
            flipAngle.setPosition(flipAngle.getPosition()-0.01);
        }
    }

    protected void onTick() {
        super.onTick();
        opMode.telemetry.addData("flip:", flipper.getPosition());
        opMode.telemetry.addData("angle:", flipAngle.getPosition());
        opMode.telemetry.update();
    }
}
