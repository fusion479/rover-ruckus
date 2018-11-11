package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoverRuckus.Constants;
import org.firstinspires.ftc.teamcode.RoverRuckus.Vision;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.SampleArm;
import org.firstinspires.ftc.teamcode.hardware.tankdrive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;



/**
 * HardwareTank is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Relic Recovery.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareMain {

    /* Mechanisms */
    public Drivetrain drivetrain;
    public Vision vision;
    public SampleArm arm;
    public Lift lift;

    public HardwareMain(){
        drivetrain = new Drivetrain();
        vision = new Vision();
        arm = new SampleArm();
        lift = new Lift();
    }
    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        drivetrain.encoderInit();
        vision.goldAlignInit(hwMap);
        arm.init(hwMap);
        lift.init(hwMap);
    }

    public void land(){
        lift.liftToPos(Constants.hookHeight-Constants.robotHeight);
        drivetrain.strafeToPos(0.5,2,3);
    }

    public void driveToSample(){
        drivetrain.driveToPos(0.5, Constants.landerToSample-Constants.robotLength,
                Constants.landerToSample-Constants.robotLength, 5);
        drivetrain.strafeToPos(0.5,15,5);
    }

    public void sampleStrafe(){
        if (vision.aligned()){
            drivetrain.driveToPos(0.5,2,2,3);
            drivetrain.driveToPos(0.5,-2,-2,3);
            drivetrain.strafeToPos(0.5,-34,5);
        }
        else {
            drivetrain.strafeToPos(0.5, -17, 5);
            if (vision.aligned()) {
                drivetrain.driveToPos(0.5, 2, 2, 3);
                drivetrain.driveToPos(0.5, -2, -2, 3);
                drivetrain.strafeToPos(0.5, -17, 5);
            } else {
                drivetrain.strafeToPos(0.5, -17, 5);
                if (vision.aligned()) {
                    drivetrain.driveToPos(0.5, 2, 2, 3);
                    drivetrain.driveToPos(0.5, -2, -2, 3);
                }
            }
        }
        drivetrain.strafeToPos(0.5,9,5);
    }

    public void markerCloseCorner(){
        drivetrain.turn(0.3,45,3);
        drivetrain.driveToPos(0.5,72,72,5);
        arm.armDown();
    }

    public void markerFarCorner(){
        drivetrain.turn(0.3,-135,3);
        drivetrain.driveToPos(0.5,72,72,5);
        arm.armDown();
    }

    public void park(){
        drivetrain.driveToPos(0.75,120,120,10);
    }
}

