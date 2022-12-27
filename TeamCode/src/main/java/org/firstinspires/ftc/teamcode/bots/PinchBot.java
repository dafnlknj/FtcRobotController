package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class PinchBot extends OdometryBot {
    public Servo pinch = null;
    public Servo scorer = null;

    //two positions of the wobble servo
    final double pinchClosed = 0.05;
    final double pinchOpened = 0.25;

    boolean isOpen = true;
    boolean isScoring = true;

    long lastToggleDone = 0;
    long timeSinceToggle = 0;
    long lastToggleDone2 = 0;
    long timeSinceToggle2 = 0;

    public PinchBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        pinch = hwMap.get(Servo.class, "pinch");
        pinch.setPosition(pinchOpened);
        scorer = hwMap.get(Servo.class, "scorer");
        scorer.setPosition(0.5);
    }

    //call openPinch() to open the arm
    public void openPinch() {
        pinch.setPosition(pinchOpened);
    }

    //call closeArm() to close the arm
    public void closePinch() {
        pinch.setPosition(pinchClosed);
    }

    public void togglePinch(boolean button) {
        timeSinceToggle = System.currentTimeMillis() - lastToggleDone;
        if (button && timeSinceToggle > 300) {
            if (isOpen) {
                pinch.setPosition(pinchClosed);
                isOpen = false;
                lastToggleDone = System.currentTimeMillis();
            } else if (!isOpen) {
                pinch.setPosition(pinchOpened);
                isOpen = true;
                lastToggleDone = System.currentTimeMillis();
            }
        }
//        opMode.telemetry.addData("lastToggle", timeSinceToggle);
//        opMode.telemetry.update();
    }

    public void controlScorer(boolean up, boolean down) {
        if (up) {
            scorer.setPosition(scorer.getPosition()+0.01);
        } else if (down) {
            scorer.setPosition(scorer.getPosition()-0.01);
        }
    }

    public void toggleScorer(boolean button) {
        timeSinceToggle2 = System.currentTimeMillis() - lastToggleDone2;
        if (button && timeSinceToggle2 > 300) {
            if (isScoring) {
                scorer.setPosition(0.75);
                isScoring = false;
                lastToggleDone2 = System.currentTimeMillis();
            } else if (!isScoring) {
                scorer.setPosition(0.3);
                isScoring = true;
                lastToggleDone2 = System.currentTimeMillis();
            }
        }
//        opMode.telemetry.addData
    }
}
