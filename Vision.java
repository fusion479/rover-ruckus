package org.firstinspires.ftc.teamcode.RoverRuckus;

//import org.firstinspires.ftc.teamcode.hardware.Mechanism;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Vision {

    private GoldAlignDetector goldAlign;
    private SamplingOrderDetector order;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AW6TLL3/////AAABmZlyx8Jj9UH/gUL2Neu0K7haI1vHe8t9JqNd42HyN7gApocQoSfmih0P5dn/ZuPVLhVtH5hRkhY8xubKIiio/VhSwDCFint59TC+Z++tYx24d4bfgtQ55u/zUJDQrRzmwFOmt0eHgOSVAhdDIjKEADW8s5qQ5JtiiJ/S0jEhSHrHLTiqFAxC8tvmV8uM6UAFuRsnheMEkDk1U2Yd1ZO0S6sM4ohcJwM4fxyYocGnMbsmXDhmwERnALZIlV9Gnk5JcAaC94ditNYXeM4U6FXKDpeIAOW9bmKR3e4Wve64fOIZE9hQLfOrYnUTPY3QbqIoFCxB9JTBQBTi7dX94+3XqcLB4lRP6Y750fmO9ciYlQod";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public LinearOpMode opMode;

    public Vision(LinearOpMode opMode){
        this.opMode = opMode;
    }


    public void goldAlignInit(HardwareMap hwMap){
        goldAlign = new GoldAlignDetector();
        goldAlign.init(hwMap.appContext, CameraViewDisplay.getInstance());
        goldAlign.useDefaults();

        // Optional Tuning
        goldAlign.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldAlign.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldAlign.downscale = 0.4; // How much to downscale the input frames

        goldAlign.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //goldAlign.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldAlign.maxAreaScorer.weight = 0.005;

        goldAlign.ratioScorer.weight = 5;
        goldAlign.ratioScorer.perfectRatio = 1.0;

        goldAlign.enable();
    }
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod(HardwareMap hwMap) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        tfod.activate();
    }

    public void sampleInit(HardwareMap hwMap) {

        order = new SamplingOrderDetector();
        order.init(hwMap.appContext, CameraViewDisplay.getInstance());
        order.useDefaults();

        order.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        order.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        order.maxAreaScorer.weight = 0.001;

        order.ratioScorer.weight = 15;
        order.ratioScorer.perfectRatio = 1.0;

        order.enable();
    }


    public boolean aligned(){
        return goldAlign.getAligned();
    }

    public String goldPos;

    public String getTensorFlow() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
            for (Recognition obj : updatedRecognitions) {
                if (obj.getLabel().equals(LABEL_GOLD_MINERAL) && obj.getTop() > 20) {
                    opMode.telemetry.addData("label=" + obj.getLabel(),
                            " left=" + obj.getLeft() +
                                    " right=" + obj.getRight() +
                                    " top=" + obj.getTop() +
                                    " confidence=" + obj.getConfidence());
                    if (obj.getLeft() < 200) {
                        opMode.telemetry.addData("position", "left");
                        opMode.telemetry.update();
                        goldPos = "LEFT";
                    } else {
                        if (obj.getLeft() > 700) {
                            opMode.telemetry.addData("position", "right");
                            opMode.telemetry.update();
                            goldPos = "RIGHT";
                        } else {
                            opMode.telemetry.addData("position", "center");
                            opMode.telemetry.update();
                            goldPos = "CENTER";
                        }
                    }
                    return goldPos;
                }
            }
        }
        return "Hello";
    }


    public double getGoldXPos(){
        return goldAlign.getXPosition();
    }

    public String getOrder(){
        return order.getCurrentOrder().toString();
    }

    public void stopGoldAlign(){
        goldAlign.disable();
    }

    public void stopOrder(){
        order.disable();
    }

    public void stopTensorFlow(){
        tfod.shutdown();
    }
}
