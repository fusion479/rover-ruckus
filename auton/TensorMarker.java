package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

import java.util.List;

@Autonomous(name = "Tensor Encoder Marker", group = "Everything")
public class TensorMarker extends LinearOpMode{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AW6TLL3/////AAABmZlyx8Jj9UH/gUL2Neu0K7haI1vHe8t9JqNd42HyN7gApocQoSfmih0P5dn/ZuPVLhVtH5hRkhY8xubKIiio/VhSwDCFint59TC+Z++tYx24d4bfgtQ55u/zUJDQrRzmwFOmt0eHgOSVAhdDIjKEADW8s5qQ5JtiiJ/S0jEhSHrHLTiqFAxC8tvmV8uM6UAFuRsnheMEkDk1U2Yd1ZO0S6sM4ohcJwM4fxyYocGnMbsmXDhmwERnALZIlV9Gnk5JcAaC94ditNYXeM4U6FXKDpeIAOW9bmKR3e4Wve64fOIZE9hQLfOrYnUTPY3QbqIoFCxB9JTBQBTi7dX94+3XqcLB4lRP6Y750fmO9ciYlQod";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public HardwareMain robot = new HardwareMain(this);
    public String position = "UNKNOWN";
    public String temp;
    public void runOpMode(){
        robot.init(hardwareMap);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        tfod.activate();
        sleep(2000);
        while (!opModeIsActive()&&!isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        int objNum = 1;
                        for (Recognition obj : updatedRecognitions) {
                            telemetry.addData("Vision Obj#"+ objNum++, "label="+obj.getLabel()+
                                    " left="+obj.getLeft()+
                                    " right="+obj.getRight()+
                                    " top="+obj.getTop()+
                                    " confidence="+obj.getConfidence());
                            if (obj.getLabel().equals(LABEL_GOLD_MINERAL) && obj.getConfidence() > 0.50 && obj.getTop() < 800){
                                telemetry.addData("Found", "gold");
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        robot.lift.unlock();
        sleep(1000);
        robot.lift.land();
        sleep(500);
        robot.lift.setHook(0);
        sleep(1500);
        if (position.equals("LEFT")) {
            telemetry.addData("Path", "right");
            telemetry.update();
            robot.drivetrain.driveToPos(0.8,4);
            robot.drivetrain.turn(30, 0.5); //right
            robot.drivetrain.driveToPos(0.8, 40);
            robot.drivetrain.turn(-70, 0.5);
            robot.drivetrain.driveToPos(0.8, 33);
            robot.marker.armUp();
            sleep(500);
            robot.drivetrain.turn(-90,0.5);
//            robot.drivetrain.turn(10,0.3);
            robot.drivetrain.driveToPos(-0.8, 80);
        }
        else{
            if (position.equals("RIGHT")) {
                telemetry.addData("Path", "left");
                telemetry.update();
                robot.drivetrain.driveToPos(0.8,4);
                robot.drivetrain.turn(-30, 0.5);
                robot.drivetrain.driveToPos(0.8, 45);
                robot.drivetrain.turn(70, 0.5);
                robot.drivetrain.driveToPos(0.8, 20);
                robot.drivetrain.turn(90,0.5);
                robot.drivetrain.turn(90,0.5);
                robot.drivetrain.driveToPos(0.8,-10);
                robot.marker.armUp();
                sleep(500);
                robot.drivetrain.driveToPos(-0.8, 90);
            }
            else{
                telemetry.addData("Path", "middle");
                robot.drivetrain.driveToPos(0.8, 65);
                robot.drivetrain.turn(-90, 0.5);
                sleep(500);
                robot.marker.armUp();
                sleep(500);
                robot.drivetrain.turn(-45, 0.5);
                robot.drivetrain.driveToPos(0.8, 10);
//                robot.drivetrain.turn(10,0.3);
                robot.drivetrain.driveToPos(-0.8, 90);
            }
        }
//        robot.lift.liftDown();
    }
    private void initVuforia() {
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
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}

