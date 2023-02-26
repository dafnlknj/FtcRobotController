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

    final double grabberClosed = 0.75; //0.67
    final double grabberOpened = 0.55; //0.52

    boolean isGrabberOpen = true;

    protected boolean shouldAngleSync = true;

    ElapsedTime grabberTimer = new ElapsedTime();

    public boolean isAuto = true;

    public FlipperBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        flipper = hwMap.get(Servo.class, "flipper");
        flipAngle = hwMap.get(Servo.class, "flipAngle");
        grabber = hwMap.get(Servo.class, "grabber");
        grabberSensor = hwMap.get(DistanceSensor.class, "grabberSensor");
        if (isAuto) {
            flipper.setPosition(0.64);
            flipAngle.setPosition(0.6);
            grabber.setPosition(grabberOpened);
        } else {
            flipper.setPosition(0.4);
            flipAngle.setPosition(0.6);
            grabber.setPosition(grabberClosed);
        }
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

    public void openGrabber() {
        grabber.setPosition(grabberOpened);
        isGrabberOpen = false;
    }

    public void closeGrabber() {
        grabber.setPosition(grabberClosed);
        isGrabberOpen = true;
    }

    public void toggleGrabber(boolean button) {
        if (button && grabberTimer.milliseconds() > 300) {
            if (isGrabberOpen) {
                openGrabber();
            } else {
                closeGrabber();
            }
            grabberTimer.reset();
        }
    }

    protected void onTick() {
        super.onTick();
//        opMode.telemetry.addData("distance:", getGrabberDistance());
//        opMode.telemetry.addData("flip:", flipper.getPosition());
//        opMode.telemetry.addData("angle:", flipAngle.getPosition());
        if (shouldAngleSync) {
            if (flipper.getPosition() < 0.5) {
                flipAngle.setPosition(Math.min(flipper.getPosition() * 1.57 + 0.193, 0.85));
            } else {
                flipAngle.setPosition(0.55);
            }
        }
    }
}
