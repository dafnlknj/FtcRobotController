package org.firstinspires.ftc.teamcode.bots;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class CameraBot extends FSMBot {

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS = {
            "Junction"
    };

    final int cameraWidth = 1280;
    final int cameraHeight = 720;
    final int offsetX = 0;
    final int offsetY = 120;
    final int spacing = 16;

    public CameraBot(LinearOpMode opMode) {
        super(opMode);
    }

    private static final String VUFORIA_KEY =
            "AW3DaKr/////AAABmbYMj0zPp0oqll2pQvI8zaoN8ktPz319ETtFtBMP7b609q4wWm6yRX9OVwWnf+mXPgSC/fSdDI2uUp/69KTNAJ6Kz+sTx+9DG+mymW00Xm3LP7Xe526NP/lM1CIBsOZ2DJlQ2mqmObbDs5WR5HXyfopN12irAile/dEYkr3uIFnJ95P19NMdbiSlNQS6SNzooW0Nc8cBKWz91P020YDqC4dHSpbQvYeFgVp2VWZJC/uyvmE15nePzZ30Uq/n8pIeYWKh4+XR74RoRyabXMXFB6PZz7lgKdRMhhhBvQ5Eh21VxjE5h8ZhGw27K56XDPk63eczGTYP/FfeLvTuK4iKSNyqRLS/37kuxKn3t/dlkwv1";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


    }

    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    //during runOpMode
    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initVuforia();
        initTfod();

        if(tfod != null) {
            tfod.activate();

            //adjust if longer distance not detecting
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }
    //waitForStart()

    // while opMode is active
    public void autoLock() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null){ //if there are objects detected
                opMode.telemetry.addData("# Objects Detected", updatedRecognitions.size());

                double largest = 10000000.00;
                Recognition closestJunction = null; //largest = closest
                double centerX = 100000;

                for (Recognition recognition : updatedRecognitions) { // iterate through every detection

                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    double area = width * height;
                    if (area > largest) {
                        largest = area;
                        centerX = (recognition.getLeft()+recognition.getRight())/2;
                        closestJunction = recognition;
                    }

                    if (recognition.getLabel().equals("Junction")) {
                        opMode.telemetry.addData("object:", "JUNCTION");
                    }
                }

                if (centerX < 640) { //if pole is to the left
                    opMode.telemetry.addData("Camera needs to", "move RIGHT");
                } else if (centerX > 640) { //if pole is to the right
                    opMode.telemetry.addData("Camera needs to", "move LEFT");
                } else { //if pole is locked in
                    opMode.telemetry.addData("Camera needs to", "stay STILL");
                }

                opMode.telemetry.update();
            }
        }
    }





}
