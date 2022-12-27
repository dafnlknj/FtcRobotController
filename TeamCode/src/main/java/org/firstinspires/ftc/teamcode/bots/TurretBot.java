package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class TurretBot extends FlipperBot {
    public Servo pinch = null;
    public Servo scorer = null;
    public DcMotor extender = null;
    public DcMotor turret = null;

    //two positions of the wobble servo
    final double pinchClosed = 0.05;
    final double pinchOpened = 0.25;

    boolean isPinchOpen = true;
    boolean isScoring = true;

    private long lastToggleDone = 0;
    private long lastToggleDone2 = 0;

    public TurretBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        pinch = hwMap.get(Servo.class, "pinch");
        pinch.setPosition(pinchOpened);
        scorer = hwMap.get(Servo.class, "scorer");
        scorer.setPosition(0.5);
        extender = hwMap.get(DcMotor.class, "extender");
        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret = hwMap.get(DcMotor.class, "turret");
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    protected void onTick() {
        super.onTick();
        opMode.telemetry.addData("extender:", extender.getCurrentPosition());
        //opMode.telemetry.addData("angle:", flipAngle.getPosition());
        opMode.telemetry.update();
    }

    public void controlExtender(float up, float down) {
        if (up > 0) {
            extender.setPower(0.1 * up);
        } else if (down > 0) {
            extender.setPower(-0.1 * down);
        } else {
            extender.setPower(0);
        }
    }

    public void togglePinch(boolean button) {
        long timeSinceToggle = System.currentTimeMillis() - lastToggleDone;
        if (button && timeSinceToggle > 300) {
            if (isPinchOpen) {
                pinch.setPosition(pinchClosed);
                isPinchOpen = false;
                lastToggleDone = System.currentTimeMillis();
            } else if (!isPinchOpen) {
                pinch.setPosition(pinchOpened);
                isPinchOpen = true;
                lastToggleDone = System.currentTimeMillis();
            }
        }
    }

    public void controlScorer(boolean up, boolean down) {
        if (up) {
            scorer.setPosition(scorer.getPosition()+0.01);
        } else if (down) {
            scorer.setPosition(scorer.getPosition()-0.01);
        }
    }

    public void toggleScorer(boolean button) {
        long timeSinceToggle2 = System.currentTimeMillis() - lastToggleDone2;
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
    }
}
