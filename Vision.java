package org.firstinspires.ftc.teamcode.RoverRuckus;

//import org.firstinspires.ftc.teamcode.hardware.Mechanism;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision {

    private GoldAlignDetector goldAlign;
    private SamplingOrderDetector order;

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

    public void sampleInit(HardwareMap hwMap) {
//        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

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
}
