/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

public class FSMBot extends TurretBot {

    private ElapsedTime timeSince = new ElapsedTime();
    private ElapsedTime timeSince2 = new ElapsedTime(100);
    private ElapsedTime timeSince3 = new ElapsedTime(300);
    private ElapsedTime timeSince4 = new ElapsedTime();
    private ElapsedTime timeSince5 = new ElapsedTime();

    final protected double flipperGround = 0.16;
    final protected double flipperLoadReady = 0.57;
    final protected double flipperLoading = 0.68;
    final protected double flipperClearTurret = 0.5;
    public double flipperStackHeight = 0.3;

    final protected double scorerLoading = 0.105; //0.105 for "normal" servos?
    protected double scorerScoreReady = 0.83; //0.83
    protected double scorerScoring = 0.91; //0.91

    private boolean shouldGrabCone = false;
    private boolean shouldScoreCone = false;
    public boolean readyToGrab = false;
    protected boolean loadingReadyTrigger = false;
    public boolean loadingStateTrigger = false;
    private boolean drivingStateTrigger = false;
    private boolean clearConeTrigger = false;
    private int firstTimeReady = 0;

    private int heightIndex = 2;

    public int coneConeState = 0;

    public int scoredCones = 0;
    protected boolean alreadyCounted = false;

    public enum ConeState {
        INIT_READY,
        READY,
        GRAB_CONE,
        LOADING_READY,
        LOADING,
        LOADING_DONE,
        EXTENDING_STAGE_1,
        EXTENDING_STAGE_2,
        EXTENDING_STAGE_3,
        SCORING,
        DROPPING,
        DRIVING,
        CLEAR_CONE
    }

    public ConeState coneState = ConeState.INIT_READY;

    public FSMBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        timeSince.reset();
        setDropHeight();
        if (isAuto) {
            coneConeState = 1;
        } else {
            coneConeState = 0;
        }
    }

    public void selectDropHeight(boolean up, boolean down, boolean up2, boolean down2) {
        if ((up || up2) && timeSince2.milliseconds() > 200 && heightIndex < 2) {
            heightIndex++;
            timeSince2.reset();
            setDropHeight();
        } else if ((down || down2) && timeSince2.milliseconds() > 200 && heightIndex > 0) {
            heightIndex--;
            timeSince2.reset();
            setDropHeight();
        }
    }

    private void setDropHeight() {
        switch (heightIndex) {
            case 0:
                maxExtension = lowExtension;
                scorerScoreReady = 0.88;
                scorerScoring = 0.88;
                break;
            case 1:
                maxExtension = mediumExtension;
                scorerScoreReady = 0.88;
                scorerScoring = 0.88;
                break;
            case 2:
                maxExtension = highExtension;
                scorerScoreReady = 0.855;
                scorerScoring = 0.855;
                break;
        }
    }

    public void resetTurretZero(boolean button) {
        if (button) {
            turretZero = turret.getCurrentPosition();
        }
    }

    public void setNewLoadingExtension(boolean button) {
        if (button) {
            loadingExtension = extender.getCurrentPosition();
        }
    }

    public void resetExtension(boolean button) {
        if (button) {
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void clearTurretRotation(boolean button) {
        if (button) {
            turretSet = 0;
        }
    }

    public void waitForState(ConeState state) {
        RobotLog.d("AUTO: wait for state started");
        while (opMode.opModeIsActive() && state != coneState) {
            if (opMode.opModeIsActive()) {
                RobotLog.d("WAIT: opModeIsActive is TRUE");
            } else {
                RobotLog.d("WAIT: opModeIsActive is FALSE");
            }
            if (state != coneState) {
                RobotLog.d(String.format("WAIT: state is %s != %s", state.toString(), coneState.toString()));
            } else {
                RobotLog.d("WAIT: state is FALSE");
            }
            sleep(50, "waiting for state");
        }
        RobotLog.d("AUTO: wait for state completed");
    }

    public void readyToGrab(boolean button, boolean button2) {
        if ((button || button2) && firstTimeReady == 1 && timeSince3.milliseconds() > 300) {
            //opMode.telemetry.addData("BRUH", true);
            coneState = ConeState.READY;
            readyToGrab = true;
            timeSince3.reset();
        } else if ((button || button2) && firstTimeReady == 0 && timeSince3.milliseconds() > 300) {
            //opMode.telemetry.addData("WTF", true);
            firstTimeReady = 1;
            readyToGrab = true;
            timeSince3.reset();
        } else {
            readyToGrab = false;
        }
    }

    public void grabberUp(boolean button, boolean button2) {
        if (button || button2) {
            coneState = ConeState.DRIVING;
            drivingStateTrigger = true;
        } else {
            drivingStateTrigger = false;
        }
    }

    public void scorerUp(float button, float button2) {
        if (button > 0 || button2 > 0) {
            coneState = ConeState.CLEAR_CONE;
            clearConeTrigger = true;
        } else {
            clearConeTrigger = false;
        }
    }

    public void stopExtender(boolean button) {
        if (button) {
            extenderSafe = false;
            extender.setPower(0);
        }
    }

    public void grabCone(boolean button) {
        shouldGrabCone = button;
    }

    public void scoreCone(boolean button, boolean button2) {
        shouldScoreCone = button || button2;
    }

    public void autoScoring(double distance, double power, double height, boolean left) {
        flipperStackHeight = height;
        waitForState(ConeState.SCORING);
        sleep(200, "problem?");
        RobotLog.d("STARTED");
        scoreCone(true, false);
        RobotLog.d("SCORED");
        //sleep(500);
        waitForState(FSMBot.ConeState.GRAB_CONE);
        //sleep(200);
//        if (left) {
//            driveToCoordinate(-24000 - distance, -87000, -90, 500, 0.1, true);
//        } else {
//            driveToCoordinate(0 + distance, 61000, -90, 750, 0.2, true);
//        }
        driveUntilDistance(distance, power);
//        if (left) {
//            driveToCoordinate(-21000 - distance - 10000, -86000, -90, 750, 0.1);
//        } else {
//            driveToCoordinate(0 + distance, 61000, -90, 750, 0.1);
//        }
//        waitForDistance(2.5);
        sleep(300);
        grabCone(true);
        sleep(200);
//        if (left) {
//            driveToCoordinate(-24000 - distance + 400, -86000, -90, 500, 0.05);
//        } else {
//            driveToCoordinate(0 + distance, 61000, -90, 750, 0.05);
//        }
//        waitForCoordinateDrive();
        waitForState(ConeState.LOADING_READY);
        loadingReadyTrigger = true;
        if (left) {
            driveToCoordinate(-20000, -86500, -90, 300, 0.1, true);
        } else {
            driveToCoordinate(21000, -85600, 90, 300, 0.1, true);
        }
        waitForState(FSMBot.ConeState.LOADING_DONE);
        loadingStateTrigger = true;
    }

    protected void doubleCheckConeScored() {
        if (!(!touchSensor.getState() && (coneState == ConeState.EXTENDING_STAGE_1 || coneState == ConeState.EXTENDING_STAGE_2 || coneState == ConeState.SCORING))) {
            timeSince4.reset();
        }
        if (timeSince4.milliseconds() > 1000 && !alreadyCounted) {
            scoredCones++;
            alreadyCounted = true;
        }
    }

    protected void driveUntilDistance(double distance, double power) {
        timeSince5.reset();
        while (opMode.opModeIsActive() && (getGrabberDistance() > distance || timeSince5.milliseconds() > 3000)) {
            sleep(10);
            driveByVector(0.18, 0, 0, 1);
        }
        driveByVector(0, 0, 0, 1);
        sleep(300);
        if (getGrabberDistance() < distance - 5) {
            while (opMode.opModeIsActive() && getGrabberDistance() < distance - 5) {
                sleep(10);
                driveByVector(-0.1, 0, 0, 1);
            }
        }
        driveByVector(0, 0, 0, 1);
    }

    protected void onTick() {
        opMode.telemetry.addData("HEIGHT:", heightIndex);
        if (coneState == ConeState.SCORING) {
            RobotLog.d(String.format("AUTO: hubba hubba %s", coneState.toString()));
        }
        doubleCheckConeScored();
        switch (coneConeState) {
            case 0:
                switch (coneState) {
                    case INIT_READY:
                        opMode.telemetry.addData("init ready", readyToGrab);
                        if (readyToGrab) {
                            RobotLog.d("MAN: init ready");

                            flipper.setPosition(0.4);
                            openGrabber();

                            scorer.setPosition(0.4);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.READY;
                        }
                        break;
                    case READY:
                        opMode.telemetry.addData("ready", readyToGrab);
                        if (readyToGrab) {
                            RobotLog.d("MAN: ready");

                            flipper.setPosition(flipperGround);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.GRAB_CONE;
                        }
                        break;
                    case GRAB_CONE:
                        opMode.telemetry.addData("grab cone", readyToGrab);
                        if (shouldGrabCone) {
                            RobotLog.d("MAN: grab cone");

                            flipper.setPosition(flipperGround);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING_READY;
                            timeSince.reset();
                        }
                        break;
                    case LOADING_READY:
                        if (timeSince.milliseconds() > 100) {
                            RobotLog.d("MAN: loading ready");

                            flipper.setPosition(flipperLoadReady);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING;
                            timeSince.reset();
                        }
                        break;
                    case LOADING:
                        if (timeSince.milliseconds() > 750) {
                            RobotLog.d("MAN: loading");

                            flipper.setPosition(flipperLoading);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING_DONE;
                            timeSince.reset();
                        }
                        break;
                    case LOADING_DONE:
                        if (!touchSensor.getState() || timeSince.milliseconds() > 2000) {
                            RobotLog.d("MAN: loading done");

                            flipper.setPosition(flipperLoading);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.EXTENDING_STAGE_1;
                            timeSince.reset();
                        }
                        break;
                    case EXTENDING_STAGE_1:
                        if (timeSince.milliseconds() > 50) {
                            RobotLog.d("MAN: extending stage 1");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.EXTENDING_STAGE_2;
                            timeSince.reset();
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (timeSince.milliseconds() > 500) {
                            RobotLog.d("MAN: extending stage 2");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.SCORING;
                        }
                        break;
                    case SCORING:
                        if (shouldScoreCone) {
                            RobotLog.d("MAN: scoring");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoring);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.DROPPING;
                            timeSince.reset();
                        }
                        break;
                    case DROPPING:
                        if (timeSince.milliseconds() > 200) {
                            RobotLog.d("MAN: dropping");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            openPinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.DRIVING;
                            timeSince.reset();
                        }
                        break;
                    case DRIVING:
                        if (timeSince.milliseconds() > 600 || drivingStateTrigger) {
                            drivingStateTrigger = false;
                            RobotLog.d(String.format("MAN: driving %.2f", timeSince.milliseconds()));

                            flipper.setPosition(0.5);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.READY;
                        }
                        break;
                    case CLEAR_CONE:
                        if (drivingStateTrigger) {
                            drivingStateTrigger = false;
                            RobotLog.d("MAN: clearing cone");

                            flipper.setPosition(flipperGround);
                            openGrabber();

                            scorer.setPosition(0.4);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.READY;
                        }
                        break;
                }
                break;
            case 1:
                switch (coneState) {
                    case INIT_READY:
                        opMode.telemetry.addData("init ready", readyToGrab);
                        if (readyToGrab) {
                            readyToGrab = false;
                            RobotLog.d("AUTO: init ready");

                            flipper.setPosition(0.64);
                            openGrabber();

                            scorer.setPosition(0.5);
                            extenderTargetPosition = loadingExtension;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING_DONE;
                        }
                        break;
                    case READY:
                        if (timeSince.milliseconds() > 800) {
                            RobotLog.d("AUTO: ready");

                            flipper.setPosition(flipperStackHeight);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.GRAB_CONE;
                        }
                        break;
                    case GRAB_CONE:
                        opMode.telemetry.addData("grab cone", readyToGrab);
                        if (shouldGrabCone) {
                            shouldGrabCone = false;
                            shouldAngleSync = false;
                            RobotLog.d("AUTO: grab cone");

                            flipper.setPosition(flipperStackHeight);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING_READY;
                            timeSince.reset();
                        }
                        break;
                    case LOADING_READY:
                        if (loadingReadyTrigger) {
                            loadingReadyTrigger = false;
                            RobotLog.d("AUTO: loading ready");

                            flipper.setPosition(flipperLoadReady);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING;
                            timeSince.reset();
                            shouldAngleSync = true;
                        }
                        break;
                    case LOADING:
                        if (timeSince.milliseconds() > 300) {
                            RobotLog.d("AUTO: loading");

                            flipper.setPosition(flipperLoading);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING_DONE;
                            timeSince.reset();
                        }
                        break;
                    case LOADING_DONE:
                        if (!touchSensor.getState() || timeSince.milliseconds() > 500) {
                            loadingStateTrigger = false;
                            RobotLog.d("AUTO: loading done");

                            flipper.setPosition(0.64);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.EXTENDING_STAGE_1;
                            timeSince.reset();
                        }
                        break;
                    case EXTENDING_STAGE_1:
                        if (timeSince.milliseconds() > 20) {
                            RobotLog.d("AUTO: extending stage 1");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(0.4);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.EXTENDING_STAGE_2;
                            timeSince.reset();
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (extender.getCurrentPosition() > 1000) {
                            RobotLog.d("AUTO: extending stage 2");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(0.4);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.EXTENDING_STAGE_3;
                        }
                        break;
                    case EXTENDING_STAGE_3:
                        if (extender.getCurrentPosition() > maxExtension - 100) {
                            RobotLog.d("AUTO: extending stage 3");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.SCORING;
                        }
                        break;
                    case SCORING:
                        RobotLog.d(String.format("AUTO: scoring wait, %d", extender.getCurrentPosition()));
                        if (shouldScoreCone) {
                            shouldScoreCone = false;
                            RobotLog.d("AUTO: scoring");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoring);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.DROPPING;
                            timeSince.reset();
                        }
                        break;
                    case DROPPING:
                        if (timeSince.milliseconds() > 300) {
                            RobotLog.d("AUTO: dropping");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            openPinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.READY;
                            timeSince.reset();
                        }
                        break;
                    case DRIVING:
                        if (timeSince.milliseconds() > 300 || drivingStateTrigger) {
                            drivingStateTrigger = false;
                            RobotLog.d("AUTO: driving");

                            flipper.setPosition(0.5);
                            openGrabber();

                            scorer.setPosition(0.3);
                            extenderTargetPosition = 0;
                            openPinch();
                            turretTargetPosition = turretZero;
                        }
                        break;
                }
                break;
            case 2:
                switch (coneState) {
                    case INIT_READY:
                        opMode.telemetry.addData("init ready", readyToGrab);
                        if (readyToGrab) {
                            RobotLog.d("MAN: init ready");

                            flipper.setPosition(0.4);
                            openGrabber();

                            scorer.setPosition(0.4);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.READY;
                        }
                        break;
                    case READY:
                        opMode.telemetry.addData("ready", readyToGrab);
                        if (readyToGrab) {
                            RobotLog.d("MAN: ready");

                            flipper.setPosition(flipperGround);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.GRAB_CONE;
                        }
                        break;
                    case GRAB_CONE:
                        opMode.telemetry.addData("grab cone", readyToGrab);
                        if (shouldGrabCone || getGrabberDistance() < 6) {
                            RobotLog.d("MAN: grab cone");

                            flipper.setPosition(flipperGround);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING_READY;
                            timeSince.reset();
                        }
                        break;
                    case LOADING_READY:
                        if (timeSince.milliseconds() > 100) {
                            RobotLog.d("MAN: loading ready");

                            flipper.setPosition(flipperLoadReady);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING;
                            timeSince.reset();
                        }
                        break;
                    case LOADING:
                        if (timeSince.milliseconds() > 750) {
                            RobotLog.d("MAN: loading");

                            flipper.setPosition(flipperLoading);
                            closeGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.LOADING_DONE;
                            timeSince.reset();
                        }
                        break;
                    case LOADING_DONE:
                        if (!touchSensor.getState() || timeSince.milliseconds() > 2000) {
                            RobotLog.d("MAN: loading done");

                            flipper.setPosition(flipperLoading);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.EXTENDING_STAGE_1;
                            timeSince.reset();
                        }
                        break;
                    case EXTENDING_STAGE_1:
                        if (timeSince.milliseconds() > 50) {
                            RobotLog.d("MAN: extending stage 1");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.EXTENDING_STAGE_2;
                            timeSince.reset();
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (timeSince.milliseconds() > 500) {
                            RobotLog.d("MAN: extending stage 2");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.SCORING;
                        }
                        break;
                    case SCORING:
                        if (shouldScoreCone) {
                            RobotLog.d("MAN: scoring");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoring);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.DROPPING;
                            timeSince.reset();
                        }
                        break;
                    case DROPPING:
                        if (timeSince.milliseconds() > 200) {
                            RobotLog.d("MAN: dropping");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(scorerScoreReady);
                            extenderTargetPosition = maxExtension;
                            openPinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.DRIVING;
                            timeSince.reset();
                        }
                        break;
                    case DRIVING:
                        if (timeSince.milliseconds() > 300 || drivingStateTrigger) {
                            drivingStateTrigger = false;
                            RobotLog.d("MAN: driving");

                            flipper.setPosition(0.5);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.READY;
                        }
                        break;
                }
                break;
        }
        super.onTick();
    }
}
