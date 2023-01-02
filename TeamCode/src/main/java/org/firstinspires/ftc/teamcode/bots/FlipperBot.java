package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class FlipperBot extends OdometryBot {
    public Servo flipper = null; //
    public Servo flipAngle = null; // range: 0.38-0.85
    public Servo grabber = null;
    protected DistanceSensor grabberSensor = null;

    final double grabberClosed = 0.6;
    final double grabberOpened = 0.45; //0.3

    boolean isGrabberOpen = true;

    ElapsedTime grabberTimer = new ElapsedTime();

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
        grabber = hwMap.get(Servo.class, "grabber");
        grabber.setPosition(grabberOpened);
        grabberSensor = hwMap.get(DistanceSensor.class, "grabberSensor");
    }

    public double getGrabberDistance() {
        return grabberSensor.getDistance(DistanceUnit.CM);
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
            flipAngle.setPosition(0.6);
        }
    }

    public void flipperToLoading(boolean button) {
        if (button) {
            flipper.setPosition(0.55);
        }
    }

    public void controlAngle(boolean up, boolean down) {
        if (up) {
            flipAngle.setPosition(flipAngle.getPosition()+0.01);
        } else if (down) {
            flipAngle.setPosition(flipAngle.getPosition()-0.01);
        }
    }

    public void grabberConeCheck() {
        if (getGrabberDistance() < 2 && !isGrabberOpen && flipper.getPosition() < 0.5) {
            grabber.setPosition(grabberClosed);
            isGrabberOpen = true;
        }
    }

    public void toggleGrabber(boolean button) {
        if (button && grabberTimer.milliseconds() > 300) {
            if (isGrabberOpen) {
                grabber.setPosition(grabberOpened);
                isGrabberOpen = false;
            } else {
                grabber.setPosition(grabberClosed);
                isGrabberOpen = true;
            }
            grabberTimer.reset();
        }
    }

    protected void onTick() {
        super.onTick();
        opMode.telemetry.addData("flip:", flipper.getPosition());
        opMode.telemetry.addData("angle:", flipAngle.getPosition());
//        opMode.telemetry.update();
    }
}
