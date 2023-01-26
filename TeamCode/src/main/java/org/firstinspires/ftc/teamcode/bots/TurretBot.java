package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class TurretBot extends FlipperBot {
    public Servo pinch = null;
    public Servo scorer = null;
    public DcMotor extender = null; //0-975
    public DcMotor turret = null;
    public DigitalChannel touchSensor = null;

    //two positions of the wobble servo
    final double pinchClosed = 0.45;
    final double pinchOpened = Math.min(pinchClosed + 0.22, 1); // diff: 0.22
    final protected int minExtension = 0;
    protected int maxExtension = 2000;
    final protected int loadingExtension = 255;
    protected int turretZero = 0;
    public int turretSet = 0;

    protected int extenderTargetPosition = 0;
    protected int turretTargetPosition = turretZero;

    boolean isPinchOpen = true;
    boolean isScoring = true;
    boolean extenderSafe = true;

    private long lastToggleDone = 0;
    private long lastToggleDone2 = 0;

    public TurretBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        pinch = hwMap.get(Servo.class, "pinch");
        scorer = hwMap.get(Servo.class, "scorer");
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
        touchSensor = hwMap.get(DigitalChannel.class, "touch");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
        if (isAuto) {
            pinch.setPosition(pinchClosed);
            scorer.setPosition(0.25);
        } else {
            pinch.setPosition(pinchOpened);
            scorer.setPosition(0.11);
        }
    }

    protected void onTick() {
        super.onTick();
        opMode.telemetry.addData("extender:", extender.getCurrentPosition());
        opMode.telemetry.addData("turret", turret.getCurrentPosition());
        opMode.telemetry.addData("scorer:", scorer.getPosition());
        if (extenderSafe) {
            extenderRunToPosition(extenderTargetPosition, 0.7);
        }
        turretRunToPosition(turretTargetPosition, 0.5);
        //opMode.telemetry.update();
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
            extenderTargetPosition = loadingExtension;
            turretRunToPosition(0, 0.3);
        }
    }

    public void goToScoringPosition(boolean button) {
        if (button) {
            scorer.setPosition(0.57);
            flipper.setPosition(0.4);
            flipAngle.setPosition(Math.min(flipper.getPosition() * 1.96 + 0.161, 0.85));
            extenderTargetPosition = maxExtension;
        }
    }

    public void controlExtender(float up, float down) {
        if (up > 0 && extender.getCurrentPosition() < maxExtension) {
            extenderTargetPosition = (int)(extenderTargetPosition + up * 50);
        } else if (down > 0 && extender.getCurrentPosition() > minExtension) {
            extenderTargetPosition = (int)(extenderTargetPosition - down * 50);
        }
    }

    public void controlTurret(boolean left, boolean right) {
        if (left) {
            turretTargetPosition = turretSet - 11;
            turretSet = turretTargetPosition;
        } else if (right) {
            turretTargetPosition = turretSet + 11;
            turretSet = turretTargetPosition;
        }
    }

    protected void openPinch() {
        pinch.setPosition(pinchOpened);
        isPinchOpen = false;
    }

    protected void closePinch() {
        pinch.setPosition(pinchClosed);
        isPinchOpen = true;
    }

    public void togglePinch(boolean button) {
        long timeSinceToggle = System.currentTimeMillis() - lastToggleDone;
        if (button && timeSinceToggle > 300) {
            if (isPinchOpen) {
                openPinch();
            } else {
                closePinch();
            }
            lastToggleDone = System.currentTimeMillis();
        }
    }

    public void controlScorer(float up, float down) {
        if (up > 0) {
            scorer.setPosition(scorer.getPosition()+0.01);
        } else if (down > 0) {
            scorer.setPosition(scorer.getPosition()-0.01);
        }
    }

    public void toggleScorer(boolean button) {
        long timeSinceToggle2 = System.currentTimeMillis() - lastToggleDone2;
        if (button && timeSinceToggle2 > 300) {
            if (isScoring) {
                scorer.setPosition(0.75);
                isScoring = false;
            } else {
                scorer.setPosition(0.4);
                isScoring = true;
            }
            lastToggleDone2 = System.currentTimeMillis();
        }
    }

    protected void extenderRunToPosition(int position, double power) {
        extender.setTargetPosition(position);
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setPower(power);
    }

    protected void turretRunToPosition(int position) {
        turretRunToPosition(position, 0.2);
    }

    protected void turretRunToPosition(int position, double power) {
        turret.setTargetPosition(position);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }
}
