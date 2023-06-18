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
    private ElapsedTime manTimer = new ElapsedTime();

    protected double flipperGround = 0.14; //0.16
    final protected double flipperLoadReady = 0.55; //0.55
    final protected double flipperLoading = 0.66; //0.66
    final protected double flipperClearTurret = 0.48;
    public double flipperStackHeight = 0.28;

    final protected double scorerLoading = 0.125; //0.105 for "normal" servos?
    protected double scorerScoreReady = 0.83; //0.83
    protected double scorerScoring = 0.91; //0.91

    private boolean shouldGrabCone = false;
    private boolean shouldScoreCone = false;
    public boolean readyToGrab = false;
    protected boolean loadingReadyTrigger = false;
    public boolean loadingStateTrigger = false;
    private boolean drivingStateTrigger = false;
    private boolean clearConeTrigger = false;
    private boolean manualLoad = false;
    private int firstTimeReady = 0;

    public int heightIndex = 2;
    private int flipperHeightIndex = 0;

    public int coneConeState = 0;
    private int manAutoStage = 0;

    public int scoredCones = 0;
    protected boolean alreadyCounted = false;

    public enum ConeState {
        INIT_READY,
        READY,
        READY_2,
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

    /**
     * Adjusts heightIndex variable (used in FSM to control extension height) with support
     * for input from both controllers.
     */
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
    /**
     * Updates relevant extension variables depending on current heightIndex.
     */
    public void setDropHeight() {
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

    public void selectFlipperHeight(boolean up, boolean down, boolean up2, boolean down2) {
        if ((up || up2) && timeSince2.milliseconds() > 200 && flipperHeightIndex < 4) {
            flipperHeightIndex++;
            timeSince2.reset();
            setFlipperHeight();
        } else if ((down || down2) && timeSince2.milliseconds() > 200 && flipperHeightIndex > 0) {
            flipperHeightIndex--;
            timeSince2.reset();
            setFlipperHeight();
        }
    }

    private void setFlipperHeight() {
        switch (flipperHeightIndex) {
            case 0:
                flipperGround = 0.16;
                break;
            case 1:
                flipperGround = 0.21;
                break;
            case 2:
                flipperGround = 0.26;
                break;
            case 3:
                flipperGround = 0.32;
                break;
            case 4:
                flipperGround = 0.39;
                break;
        }
    }
    /**
     * Resets the center position of the turret (turretZero).
     */
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
    /**
     * Resets the slide extension with the current position as the new zero.
     */
    public void resetExtension(boolean button) {
        if (button) {
            extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    /**
     * Resets the automatic turret rotation to zero.
     */
    public void clearTurretRotation(boolean button) {
        if (button) {
            turretSet = 0;
        }
    }
    /**
     * Waits for a certain state in the FSM to be reached. All the RobotLog methods are for debugging
     * since this method can be bug-prone at times.
     * @param state state to be reached
     */
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
    /**
     * Sets the current state to READY. Includes an if statement to run slightly differently the first time.
     * Use case: all components of the robot prepare to grab a cone,
     * flipper - down/ground level
     * grabber - open
     * scorer - loading
     * extender - loading
     * turret - center
     */
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
    /**
     * Sets the current state to DRIVING.
     * Use case: brings the grabber up so the robot can navigate the field easier.
     */
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
    /**
     * Really quick and easy "kill switch" to stop the slides.
     * WARNING: breaks the FSM, do not use for competition.
     */
    public void stopExtender(boolean button) {
        if (button) {
            extenderSafe = false;
            extender.setPower(0);
        }
    }
    /**
     * Triggers the transition from the READY_2 to GRAB_CONE state.
     * Use case: grabs a cone and starts the automatic loading process.
     */
    public void grabCone(boolean button) {
        shouldGrabCone = button;
    }
    /**
     * Triggers the transition from the EXTENDING_STAGE_2 to SCORING state.
     * Use case: drops a cone and starts the automatic retraction process.
     */
    public void scoreCone(boolean button, boolean button2) {
        shouldScoreCone = button || button2;
    }
    /**
     * Triggers the transition from the LOADING to LOADING_DONE state. Replaces the use of a touch sensor
     * to detect if a cone has been successfully loaded. Use case: starts the automatic extension process.
     */
    public void manLoad(boolean button, boolean button2) {
        manualLoad = button || button2;
    }
    /**
     * Old method used in autonomous.
     */
    public void autoScoring(double distance, double power, double height, boolean left) {
        flipperStackHeight = height;
        waitForState(ConeState.SCORING);
        sleep(500, "problem?");
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
        //driveUntilDistance(distance, power);
//        if (left) {
//            driveToCoordinate(-21000 - distance - 10000, -86000, -90, 750, 0.1);
//        } else {
//            driveToCoordinate(0 + distance, 61000, -90, 750, 0.1);
//        }
//        waitForDistance(2.5);
        sleep(150);
        grabCone(true);
        sleep(250);
//        if (left) {
//            driveToCoordinate(-24000 - distance + 400, -86000, -90, 500, 0.05);
//        } else {
//            driveToCoordinate(0 + distance, 61000, -90, 750, 0.05);
//        }
//        waitForCoordinateDrive();
        waitForState(ConeState.LOADING_READY);
        loadingReadyTrigger = true;
        if (left) {
            driveToCoordinate(-20000, -83500, -93, 600, 1, 0.1, true);
        } else {
            driveToCoordinate(22500, -85600, 90, 300, 0.1, true);
        }
        waitForState(FSMBot.ConeState.LOADING_DONE);
        loadingStateTrigger = true;
    }
    /**
     * Simplified version of {@link #autoScoringNoDist(double, double, double, int, double, double, int, boolean, boolean)}.
     * @param distance adjustable distance from wall, relative to a preset position (check method for specific)
     * @param power max power for driving
     * @param height stack height (servo position)
     * @param rotation turret rotation
     * @param left is this method being used in a left side auto?
     * @param last is this the last time?
     */
    public void autoScoringNoDist(double distance, double power, double height, int rotation, boolean left, boolean last) {
        autoScoringNoDist(distance, 20000, -85600, 2, power, height, rotation, left, last);
    }
    /**
     * Newer method that no longer relies on the distance sensor.
     * @param distance adjustable distance from wall, relative to a preset position (check method for specific)
     * @param drop x-coordinate of dropping position
     * @param yDrop y-coordinate of dropping position
     * @param index extension height (heightIndex)
     * @param power max power for driving
     * @param height stack height (servo position)
     * @param rotation turret rotation
     * @param left is this method being used in a left side auto?
     * @param last is this the last time?
     */
    public void autoScoringNoDist(double distance, double drop, double yDrop, int index, double power, double height, int rotation, boolean left, boolean last) {
        flipperStackHeight = height;
        waitForState(ConeState.SCORING);
        sleep(400, "problem?");
        RobotLog.d("STARTED");
        scoreCone(true, false);
        RobotLog.d("SCORED");
        //sleep(500);
        waitForState(FSMBot.ConeState.GRAB_CONE);
        heightIndex = index;
        setDropHeight();
        turretSet = rotation;
        sleep(200);
        if (left) {
            driveToCoordinate(-21000 - distance, -84500, -90, 500, power, true);
//            timeSince5.reset();
//            while (opMode.opModeIsActive() && timeSince5.milliseconds() < 500) {
//                sleep(10);
//                driveByVector(0.3, 0, 0, 1);
//            }
//            driveByVector(0, 0, 0, 1);
        } else {
            driveToCoordinate(19000 + distance, -85600, 90, 700, power, true);
        }
//        if (left) {
//            driveToCoordinate(-21000 - distance - 10000, -86000, -90, 750, 0.1);
//        } else {
//            driveToCoordinate(0 + distance, 61000, -90, 750, 0.1);
//        }
        if (last) {
            autoOffset = 0;
        } else {
            autoOffset = 5;
        }
        waitForCoordinateDrive();
        sleep(100);
        grabCone(true);
        sleep(250);
//        if (left) {
//            driveToCoordinate(-24000 - distance + 400, -86000, -90, 500, 0.05);
//        } else {
//            driveToCoordinate(0 + distance, 61000, -90, 750, 0.05);
//        }
//        waitForCoordinateDrive();
        waitForState(ConeState.LOADING_READY);
        loadingReadyTrigger = true;
        if (left) {
            driveToCoordinate(-22000, -84500, -90, 500, 1, power, true);
        } else {
            driveToCoordinate(drop, yDrop, 90, 500, 1, power, true); //-85600
        }
        waitForState(FSMBot.ConeState.LOADING_DONE);
        loadingStateTrigger = true;
    }
    public void manAuto(boolean button) {
        if (button) {
            xBlue = -5600;
            yBlue = 0;
            thetaRAD = Math.toRadians(getAngle());
            flipperStackHeight = 0.39;
            turretSet = -415;
            coneConeState = 1;
            coneState = ConeState.GRAB_CONE;
            grabCone(true);
            sleep(250);
            waitForState(ConeState.LOADING_READY);
            loadingReadyTrigger = true;
            driveToCoordinate(0, 0, -90, 500, 1, 0.3, true);
            waitForState(FSMBot.ConeState.LOADING_DONE);
            loadingStateTrigger = true;
            autoScoringMan(4000, 0.3, 0.32, true, false); //0.26
            autoScoringMan(3200, 0.3, 0.26, true, false); //0.26
            autoScoringMan(2500, 0.3, 0.21, true, false);
            autoScoringMan(2800, 0.3, 0.16, true, true);
            waitForState(FSMBot.ConeState.SCORING);
            sleep(500);
            scoreCone(true, false);
            waitForState(FSMBot.ConeState.GRAB_CONE);
            coneConeState = 0;
            coneState = FSMBot.ConeState.DRIVING;
            driveToCoordinate(70000, 0, 0, 5000, 3, 1, false);
            waitForCoordinateDrive();
            turretSet = 350;
        }
    }
    /**
     * Runs an uninterruptible mini-FSM that scores cones onto the high junction.
     * Basically an adapted autonomous for TeleOp.
     */
    public void manAutoFSM(boolean button) {
        switch (manAutoStage) {
            case 0:
                break;
            case 1:
                xBlue = -5600;
                yBlue = 0;
                thetaRAD = Math.toRadians(getAngle());
                flipperStackHeight = 0.39;
                turretSet = -415;
                coneConeState = 1;
                autoOffset = -10;
                coneState = ConeState.GRAB_CONE;

                manTimer.reset();
                manAutoStage = 2;
                break;
            case 2:
                if (manTimer.milliseconds() > 150) {
                    grabCone(true);

                    manTimer.reset();
                    manAutoStage = 3;
                }
                break;
            case 3:
                if (manTimer.milliseconds() > 250 && ConeState.LOADING_READY == coneState) {
                    loadingReadyTrigger = true;
                    driveToCoordinate(0, 0, -90, 500, 1, 0.3, true);

                    manAutoStage = 4;
                }
                break;
            case 4:
                if (ConeState.LOADING_DONE == coneState) {
                    loadingStateTrigger = true;
                    flipperStackHeight = 0.32;

                    manAutoStage = 5;
                }
                break;
            case 5:
                scorer.setPosition(0.855);
                if (button || shouldScoreCone) {
                    scorer.setPosition(0.5);
                    scoreCone(true, false);

                    manAutoStage = 6;
                }
                break;
            case 6:
                if (ConeState.GRAB_CONE == coneState) {

                    manTimer.reset();
                    manAutoStage = 7;
                }
                break;
            case 7:
                if (manTimer.milliseconds() > 150) {
                    driveToCoordinate(1000 - 5000, 0, -90, 500, 0.3, true);
                    autoOffset = 0;

                    manAutoStage = 8;
                }
                break;
            case 8:
                if (!isCoordinateDriving) {

                    manTimer.reset();
                    manAutoStage = 9;
                }
                break;
            case 9:
                if (manTimer.milliseconds() > 70) {
                    grabCone(true);

                    manTimer.reset();
                    manAutoStage = 10;
                }
                break;
            case 10:
                if (manTimer.milliseconds() > 210 && ConeState.LOADING_READY == coneState) {
                    loadingReadyTrigger = true;
                    driveToCoordinate(0, 0, -90, 500, 1, 0.3, true);

                    manAutoStage = 11;
                }
                break;
            case 11:
                if (ConeState.LOADING_DONE == coneState) {
                    loadingStateTrigger = true;
                    autoScoringMan(3200, 0.3, 0.26, true, false); //0.26
                    autoScoringMan(2500, 0.3, 0.21, true, false);
                    autoScoringMan(2800, 0.3, 0.16, true, true);
                    waitForState(FSMBot.ConeState.SCORING);
                    sleep(500);
                    scoreCone(true, false);
                    waitForState(FSMBot.ConeState.GRAB_CONE);
                    coneConeState = 0;
                    coneState = FSMBot.ConeState.DRIVING;
                    driveToCoordinate(70000, 0, 0, 5000, 3, 1, false);
                    waitForCoordinateDrive();
                    turretSet = 350;

                    manAutoStage = 0;
                }
                break;
        }
        if (manAutoStage == 0 && button) {
            manAutoStage = 1;
//            xBlue = -5600;
//            yBlue = 0;
//            thetaRAD = Math.toRadians(getAngle());
//            flipperStackHeight = 0.39;
//            turretSet = -415;
//            coneConeState = 1;
//            coneState = ConeState.GRAB_CONE;
//            grabCone(true);
//            sleep(250);
//            waitForState(ConeState.LOADING_READY);
//            loadingReadyTrigger = true;
//            driveToCoordinate(0, 0, -90, 500, 1, 0.3, true);
//            waitForState(FSMBot.ConeState.LOADING_DONE);
//            loadingStateTrigger = true;
//            autoScoringMan(4000, 0.3, 0.32, true, false); //0.26
//            autoScoringMan(3200, 0.3, 0.26, true, false); //0.26
//            autoScoringMan(2500, 0.3, 0.21, true, false);
//            autoScoringMan(2800, 0.3, 0.16, true, true);
//            waitForState(FSMBot.ConeState.SCORING);
//            sleep(500);
//            scoreCone(true, false);
//            waitForState(FSMBot.ConeState.GRAB_CONE);
//            coneConeState = 0;
//            coneState = FSMBot.ConeState.DRIVING;
//            driveToCoordinate(70000, 0, 0, 5000, 3, 1, false);
//            waitForCoordinateDrive();
//            turretSet = 350;
        }
    }
    /**
     * Version of {@link #autoScoringNoDist(double, double, double, int, boolean, boolean)} adapted for the {@link #manAutoFSM(boolean)} method.
     */
    public void autoScoringMan(double distance, double power, double height, boolean left, boolean last) {
        flipperStackHeight = height;
        waitForState(ConeState.SCORING);
        sleep(500, "problem?");
        RobotLog.d("STARTED");
        scoreCone(true, false);
        RobotLog.d("SCORED");
        //sleep(500);
        waitForState(FSMBot.ConeState.GRAB_CONE);
        sleep(200);
        if (left) {
            driveToCoordinate(1000 - distance, 0, -90, 500, power, true);
        } else {
            driveToCoordinate(19000 + distance, -85600, 90, 700, power, true);
        }
        if (last) {
            autoOffset = 0;
        } else {
            autoOffset = 5;
        }
        waitForCoordinateDrive();
        sleep(100);
        grabCone(true);
        sleep(250);
        waitForState(ConeState.LOADING_READY);
        loadingReadyTrigger = true;
        if (left) {
            driveToCoordinate(0, 0, -90, 500, 1, power, true);
        } else {
            driveToCoordinate(20000, -85600, 90, 500, 1, power, true);
        }
        waitForState(FSMBot.ConeState.LOADING_DONE);
        loadingStateTrigger = true;
    }
    /**
     * Raises the scorer from slightly below a junction to over it.
     * Use case: helps the driver to score accurately.
     */
    public void adjustAlign(boolean button) {
        if (coneState == ConeState.SCORING) {
            if (button) {
                scorer.setPosition(scorerScoreReady);
            } else {
                scorer.setPosition(scorerScoreReady + 0.06);
            }
        }
    }

    protected void onTick() {
        opMode.telemetry.addData("stage:", manAutoStage);
        opMode.telemetry.addData("HEIGHT:", heightIndex);
//        if (coneState == ConeState.SCORING) {
//            RobotLog.d(String.format("AUTO: hubba hubba %s", coneState.toString()));
//        }
        //doubleCheckConeScored();
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
                            //shouldAngleSync = false;

                            if (flipper.getPosition() > 0.4) {
                                flipper.setPosition(flipperGround);
                            }
                            //flipAngle.setPosition(0.9);
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension;
                            openPinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.GRAB_CONE;
                            timeSince.reset();
                        }
                        break;
                    case READY_2:
                        if (timeSince.milliseconds() > 150) {
                            RobotLog.d("MAN: ready 2");
                            shouldAngleSync = true;

                            if (flipper.getPosition() > 0.4) {
                                flipper.setPosition(flipperGround);
                            }
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
                            shouldAngleSync = true;

                            if (flipper.getPosition() > 0.4) {
                                flipper.setPosition(flipperGround);
                            }
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
                        if (timeSince.milliseconds() > 200) {
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
                        if (manualLoad) { //!touchSensor.getState() || timeSince.milliseconds() > 400
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

                            scorer.setPosition(scorerScoring + 0.06);
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

                            scorer.setPosition(scorerScoreReady + 0.06);
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
                            flipAngle.setPosition(0.9);
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
                            extenderTargetPosition = loadingExtension + autoOffset;
                            closePinch();
                            turretTargetPosition = turretZero;

                            timeSince.reset();
                            coneState = ConeState.LOADING_DONE;
                        }
                        break;
                    case READY:
                        if (timeSince.milliseconds() > 400) {
                            RobotLog.d("AUTO: ready");

                            flipper.setPosition(flipperStackHeight);
                            flipAngle.setPosition(Math.min(flipperStackHeight * 1.67 + 0.179, 0.95));
                            openGrabber();

                            scorer.setPosition(scorerLoading);
                            extenderTargetPosition = loadingExtension + autoOffset;
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
                            extenderTargetPosition = loadingExtension + autoOffset;
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
                            extenderTargetPosition = loadingExtension + autoOffset;
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
                            extenderTargetPosition = loadingExtension + autoOffset;
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
                            extenderTargetPosition = loadingExtension + autoOffset;
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
                            extenderTargetPosition = maxExtension - 300;
                            closePinch();
                            turretTargetPosition = turretZero;

                            coneState = ConeState.EXTENDING_STAGE_2;
                            timeSince.reset();
                        }
                        break;
                    case EXTENDING_STAGE_2:
                        if (extender.getCurrentPosition() > 600) {
                            RobotLog.d("AUTO: extending stage 2");

                            flipper.setPosition(flipperClearTurret);
                            openGrabber();

                            scorer.setPosition(0.65);
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

                            scorer.setPosition(0.65);
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

                            scorer.setPosition(0.9);
                            extenderTargetPosition = maxExtension;
                            closePinch();
                            turretTargetPosition = turretSet;

                            coneState = ConeState.DROPPING;
                            timeSince.reset();
                        }
                        break;
                    case DROPPING:
                        if (timeSince.milliseconds() > 100) {
                            RobotLog.d("AUTO: dropping");

                            flipper.setPosition(flipperClearTurret);
                            flipAngle.setPosition(Math.min(flipperStackHeight * 1.67 + 0.179, 0.95));
                            openGrabber();

                            scorer.setPosition(0.9);
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
