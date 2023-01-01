package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class TurretBot extends FlipperBot {
    public Servo pinch = null;
    public Servo scorer = null;
    public DcMotor extender = null; //0-975
    public DcMotor turret = null;

    //two positions of the wobble servo
    final double pinchClosed = 0;
    final double pinchOpened = 0.1; //0.25
    final int minExtension = 0;
    final int maxExtension = 975;
    final int loadingExtension = 650;

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

    public void handOff(boolean button) {
        if (button) {
            togglePinch(true);
            toggleGrabber(true);
        }
    }

    public void goToLoadingPosition(boolean button) {
        if (button) {
            scorer.setPosition(0);
            extenderRunToPosition(loadingExtension, 0.5);
            turretRunToPosition(0, 0.3);
        }
    }

    public void goToScoringPosition(boolean button) {
        if (button) {
            scorer.setPosition(0.57);
            flipper.setPosition(0.4);
            flipAngle.setPosition(Math.min(flipper.getPosition() * 1.96 + 0.161, 0.85));
            extenderRunToPosition(maxExtension, 0.5);
        }
    }

    public void controlExtender(float up, float down) {
        if (up > 0 && extender.getCurrentPosition() < maxExtension) {
            extender.setTargetPosition((int) (extender.getCurrentPosition() + up * 50));
            extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extender.setPower(0.2);
        } else if (down > 0 && extender.getCurrentPosition() > minExtension) {
            extender.setTargetPosition((int) (extender.getCurrentPosition() - down * 50));
            extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extender.setPower(0.2);
        } else if (!extender.isBusy()) {
            extender.setPower(0);
        }
    }

    public void controlTurret(float input) {
        if (input > 0) {
            turret.setTargetPosition((int) (turret.getCurrentPosition() + input * 15));
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.2);
        } else if (input < 0) {
            turret.setTargetPosition((int) (turret.getCurrentPosition() + input * 15));
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.2);
        } else if (!turret.isBusy()) {
            turret.setPower(0);
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
                scorer.setPosition(0.4);
                isScoring = true;
                lastToggleDone2 = System.currentTimeMillis();
            }
        }
    }

    public void extenderRunToPosition(int position, double power) {
        extender.setTargetPosition(position);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(power);
    }

    public void turretRunToPosition(int position, double power) {
        turret.setTargetPosition(position);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }
}
