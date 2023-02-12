package org.firstinspires.ftc.teamcode.bots;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.stormbots.MiniPID;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class OdometryBot extends GyroBot {

    public DcMotor horizontal = null;
    public DcMotor verticalRight = null;
    //public Servo odometryRaise = null;

    String verticalLeftEncoderName = "v1", verticalRightEncoderName = "v2", horizontalEncoderName = "h";

    public double xBlue = 0, yBlue = 0, xBlueChange = 0, yBlueChange = 0, thetaDEG = 0, previousThetaDEG = 0;
    double xRed = 0, yRed = 0, xRedChange = 0, yRedChange = 0;
    double hError = 0;

    protected double[] driveAccelerationCurve = new double[]{0.5, 0.6, 0.8, 0.9, 0.8, 0.9};

    double savedXBlue, savedYBlue, savedThetaDEG;
    public double savedStartAngle;

    final int vLDirection = 1;
    final int vRDirection = 1;
    final int hDirection = 1;
    final double diameter = 18719; // actually diameter: 285/609 = d/40000
    final double hDiameter = 24302; //diameter of horizontal encoder: 185*2/609 = hD/40000

    double vLOffset, vROffset, hOffset = 0;

    public double previousVL = 0, previousVR = 0, previousH = 0;
    double angleChange = 0;

    double drive;
    double strafe;
    double twist;
    double driveAngle;
    double thetaDifference;
    double distanceToTarget;
    long startTime;
    long elapsedTime = 0;
    public boolean isCoordinateDriving = false;
    public boolean isTurningInPlace = false;

    double globalTargetX = 0;
    double globalTargetY = 0;
    double globalTargetTheta = 0;
    int globalTolerance = 0;
    double globalMagnitude = 0;

    ElapsedTime robotLogTimer = new ElapsedTime();

    OutputStreamWriter odometryWriter;

    public OdometryBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initDriveHardwareMap(ahwMap);
        context = hwMap.appContext;
        opMode.telemetry.addData("Status", "Init Complete");
        opMode.telemetry.update();
        robotLogTimer.reset();
    }

    private void initDriveHardwareMap(HardwareMap ahwMap){

        horizontal = ahwMap.dcMotor.get(horizontalEncoderName);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalLeft = ahwMap.dcMotor.get(verticalLeftEncoderName);
//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight = ahwMap.dcMotor.get(verticalRightEncoderName);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
    }

    Context context;

    public void outputEncoders() {
//        opMode.telemetry.addData("h", horizontal.getCurrentPosition());
//        opMode.telemetry.update();
        RobotLog.d(String.format("h: %d time: %.0f", rightFront.getCurrentPosition(), robotLogTimer.milliseconds()));
    }

    public void calculateCaseThree(double vL, double vR, double h) {
        vL = vL * vLDirection;
        vR = vR * vRDirection;
        h = h * hDirection;

        double lC = vL - previousVL;
        double rC = vR - previousVR;

//        angleChange = ((lC - rC) / (Math.PI * diameter * 2) * 360);
//        angleChange = (lC - rC)/(2 * diameter);

        //angleDEG = angleDEG + angleChange;
        //thetaDEG = angleDEG;

        thetaDEG = getDeltaAngle();
        //angleChange = angleDEG - previousThetaDEG;

        hError = (lC - rC)/(2 * diameter) * hDiameter;

        double hC = h - previousH;

        xRedChange = hC + hError;
        yRedChange = (lC + rC)/2;
        //yRedChange = lC;

        xBlueChange = Math.cos(Math.toRadians(thetaDEG - 90)) * xRedChange + Math.cos(Math.toRadians(thetaDEG)) * yRedChange;
        yBlueChange = Math.sin(Math.toRadians(thetaDEG)) * yRedChange + Math.sin(Math.toRadians(thetaDEG - 90)) * xRedChange;

        xBlue = xBlue + yBlueChange;
        yBlue = yBlue + xBlueChange;

        previousVL = vL;
        previousVR = vR;
        previousH = h;
        //previousThetaDEG = angleDEG;
    }

//    public void resetOdometry(boolean button) {
//
//        if (button) {
////            vLOffset = leftFront.getCurrentPosition();
////            vROffset = rightFront.getCurrentPosition();
////            hOffset = horizontal.getCurrentPosition() + 79000;
//
//            horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            previousVL = 0;
//            previousVR = 0;
//            previousH = 0;
//
//            xBlue = 79000;
//            yBlue = 0;
//
//            thetaDEG = 0;
//        }
//    }

    protected void onTick(){
        RobotLog.d(String.format("Position, heading: %.2f, %.2f, %.2f", xBlue, yBlue, thetaDEG));

        opMode.telemetry.addData("X:", xBlue);
        opMode.telemetry.addData("Y:", yBlue);
        opMode.telemetry.addData("Theta:", thetaDEG);
        opMode.telemetry.addData("v1", verticalRight.getCurrentPosition());
        opMode.telemetry.addData("h", horizontal.getCurrentPosition());
        opMode.telemetry.addData("h diameter", (int)((thetaDEG*360)/(horizontal.getCurrentPosition() * Math.PI)));
//        opMode.telemetry.update();

        //outputEncoders();
        super.onTick();
        thetaDEG = -getDeltaAngle();
        calculateCaseThree(leftRear.getCurrentPosition() - vLOffset, -verticalRight.getCurrentPosition() - vROffset, -horizontal.getCurrentPosition() - hOffset);
        if (isCoordinateDriving) {
            driveToCoordinateUpdate(globalTargetX, globalTargetY, globalTargetTheta, globalTolerance, globalMagnitude);
        }
        if (isTurningInPlace) {
            turnInPlaceUpdate(globalTargetX, globalTargetY, globalTargetTheta, globalTolerance, globalMagnitude);
        }
    }

    public void driveToCoordinate(double xTarget, double yTarget, double targetTheta, int tolerance, double magnitude, boolean brake) {
        if (brake) {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (xBlue > xTarget) {
            distanceToTarget = - Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        } else {
            distanceToTarget = Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        }
        RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f", xBlue, yBlue, thetaDEG));
        globalTargetX = xTarget;
        globalTargetY = yTarget;
        globalTargetTheta = targetTheta;
        globalTolerance = tolerance;
        globalMagnitude = magnitude;

        isCoordinateDriving = true;

//            driveToCoordinateUpdate(xTarget, yTarget, targetTheta, tolerance, magnitude);

//            elapsedTime = System.currentTimeMillis() - startTime;
//            if (elapsedTime > 10000) {
//                break;
//            }
    }

    public void driveToCoordinateUpdate(double xTarget, double yTarget, double targetTheta, int tolerance, double magnitude) {
        MiniPID drivePID = new MiniPID(0.05, 0, 0);//i: 0.006 d: 0.06
        MiniPID twistPID = new MiniPID(0.025, 0.005, 0.03);
        drivePID.setOutputLimits(magnitude);
        twistPID.setOutputLimits(magnitude);
        thetaDifference = targetTheta - thetaDEG;
        twist = twistPID.getOutput(thetaDEG, targetTheta);
        double rawDriveAngle = Math.toDegrees(Math.atan2(xTarget - xBlue, yTarget - yBlue));
        driveAngle = -(rawDriveAngle - thetaDEG);
        magnitude = Math.min(1.0, Math.abs(drivePID.getOutput(distanceToTarget/5000, 0))*2);
        if (Math.abs(distanceToTarget) < 8000) {
            magnitude = Math.max(0.13, Math.min(1.0, Math.abs(drivePID.getOutput(distanceToTarget/5000, 0))));
        }
        if (xBlue > xTarget) {
            distanceToTarget = - Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        } else {
            distanceToTarget = Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        }
        drive = (Math.cos(Math.toRadians(driveAngle)) * magnitude);
        strafe = Math.sin(Math.toRadians(driveAngle)) * magnitude;

        driveByVector(drive, strafe, -twist, 1);
        RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f Angle: %f Drive: %f Strafe: %f Twist: %f", xBlue, yBlue, thetaDEG, driveAngle, drive, strafe, twist));
        RobotLog.d(String.format("Distance: %f Magnitude: %f", distanceToTarget, magnitude));

        if ((xTarget + tolerance > xBlue) && (xTarget - tolerance < xBlue) && (yTarget + tolerance > yBlue) && (yTarget - tolerance < yBlue) && Math.abs(thetaDifference) < 2) {
            isCoordinateDriving = false;
            driveByVector(0, 0, 0, 1);
            RobotLog.d("TARGET REACHED");
        } else {
            isCoordinateDriving = true;
        }
    }

    public void turnInPlace(double targetTheta, int tolerance, double magnitude) {
        distanceToTarget = 0;
        RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f", xBlue, yBlue, thetaDEG));
        globalTargetX = xBlue;
        globalTargetY = yBlue;
        globalTargetTheta = targetTheta;
        globalTolerance = tolerance;
        globalMagnitude = magnitude;

        isTurningInPlace = true;
    }

    public void turnInPlaceUpdate(double xTarget, double yTarget, double targetTheta, int tolerance, double magnitude) {
        MiniPID drivePID = new MiniPID(0.05, 0, 0);//i: 0.006 d: 0.06
        MiniPID twistPID = new MiniPID(0.025, 0.005, 0.03);
        drivePID.setOutputLimits(magnitude);
        twistPID.setOutputLimits(magnitude);
        thetaDifference = targetTheta - thetaDEG;
        twist = twistPID.getOutput(thetaDEG, targetTheta);
        double rawDriveAngle = Math.toDegrees(Math.atan2(xTarget - xBlue, yTarget - yBlue));
        driveAngle = rawDriveAngle - thetaDEG;
        magnitude = Math.min(1.0, Math.abs(drivePID.getOutput(distanceToTarget/5000, 0))*2);
        if (Math.abs(distanceToTarget) < 10000) {
            magnitude = Math.max(0.05, Math.min(1.0, Math.abs(drivePID.getOutput(distanceToTarget/5000, 0))));
        }
        if (xBlue > xTarget) {
            distanceToTarget = - Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        } else {
            distanceToTarget = Math.sqrt(Math.pow(xBlue - xTarget, 2) + Math.pow(yBlue - yTarget, 2));
        }
        drive = -(Math.cos(Math.toRadians(driveAngle)) * magnitude);
        strafe = Math.sin(Math.toRadians(driveAngle)) * magnitude;

        driveByVector(drive, -strafe, twist, 1);
        RobotLog.d(String.format("BlueX: %f BlueY: %f Theta: %f Angle: %f Drive: %f Strafe: %f Twist: %f", xBlue, yBlue, thetaDEG, driveAngle, drive, strafe, twist));
        RobotLog.d(String.format("Distance: %f Magnitude: %f", distanceToTarget, magnitude));

        if ((xTarget + tolerance > xBlue) && (xTarget - tolerance < xBlue) && (yTarget + tolerance > yBlue) && (yTarget - tolerance < yBlue) && Math.abs(thetaDifference) < 2) {
            isTurningInPlace = false;
            driveByVector(0, 0, 0, 1);
            RobotLog.d("TARGET REACHED");
        } else {
            isTurningInPlace = true;
        }
    }

    public void waitForCoordinateDrive() {
        while (opMode.opModeIsActive() && isCoordinateDriving) {
            sleep(10);
        }
    }

    public void waitForTurnInPlace() {
        while (opMode.opModeIsActive() && isTurningInPlace) {
            sleep(10);
        }
    }

//    public void savePosition() {
////        try {
////            odometryWriter = new FileWriter("/sdcard/FIRST/odometry positions.txt", false);
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer open failed: " + e.toString());
////        }
////        try {
////            RobotLog.d("odometryWriter.write");
////            odometryWriter.write(xBlue + "\n");
////            odometryWriter.write(yBlue + "\n");
////            odometryWriter.write(thetaDEG + "\n");
////            odometryWriter.write(getAngle() + "\n");
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer write failed: " + e.toString());
////        }
////        try {
////            RobotLog.d("odometryWriter.close");
////            odometryWriter.close();
////        } catch (IOException e) {
////            throw new RuntimeException("odometry file writer close failed: " + e.toString());
////        }
//        try {
//            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput("odometry positions.txt", Context.MODE_PRIVATE));
//
//            // write each configuration parameter as a string on its own line
//            outputStreamWriter.write(xBlue + "\n");
//            outputStreamWriter.write(yBlue + "\n");
//            outputStreamWriter.write(thetaDEG + "\n");
//            outputStreamWriter.write(getAngle() + "\n");
//
//            outputStreamWriter.close();
//        }
//        catch (IOException e) {
//            opMode.telemetry.addData("Exception", "Configuration file write failed: " + e.toString());
//        }
//
//    }

//    public void readPosition() {
//        try {
//            InputStream inputStream = context.openFileInput("odometry positions.txt");
//            if ( inputStream != null ) {
//                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
//                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
//
//                xBlue = Double.parseDouble(bufferedReader.readLine());
//                opMode.telemetry.addData("X:", xBlue);
//                yBlue = Double.parseDouble(bufferedReader.readLine());
//                opMode.telemetry.addData("Y:", yBlue);
//                opMode.telemetry.update();
//                RobotLog.d(String.format("odometry bodoo: %.2f, %.2f", xBlue, yBlue));
//                thetaDEG = Double.parseDouble(bufferedReader.readLine());
//                savedStartAngle = Double.parseDouble(bufferedReader.readLine());
//                thetaDEG = savedStartAngle;
//
//                inputStream.close();
//            }
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }
}
