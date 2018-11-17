package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoverRuckus.Constants;
import org.firstinspires.ftc.teamcode.RoverRuckus.Vision;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.SampleArm;
import org.firstinspires.ftc.teamcode.hardware.Lift;

/**
 * HardwareTank is the class that is used to define all of the hardware for a single robot. In this
 * case, the robot is: Relic Recovery.
 *
 * This class contains autonomous actions involving multiple mechanisms. These actions
 * may be common to more than one routine.
 */
public class HardwareTank {

    /* Mechanisms */
    public Drivetrain drivetrain;
    public Vision vision;
    public SampleArm arm;
    public Lift lift;
    public LinearOpMode opMode;

    public HardwareTank(LinearOpMode opMode){
        drivetrain = new Drivetrain(opMode);
        vision = new Vision();
        arm = new SampleArm();
        this.opMode = opMode;
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
        lift.unhook();
        lift.liftToPos(7);
        drivetrain.strafeToPos(0.5,2,3);
    }

    public void driveToSample(){
        drivetrain.driveToPos(0.5, Constants.landerToSample-Constants.robotLength,
                Constants.landerToSample-Constants.robotLength, 5);
        drivetrain.turn(90, 5);
        drivetrain.driveToPos(0.5, -17, -17, 5);
    }

    public void hitMineral(){
        drivetrain.turn(-90, 5);
        drivetrain.driveToPos(0.5,4,4,5);
        drivetrain.driveToPos(0.5,4,-4,5);
    }

    public void sample(){
        if(vision.aligned()){
            hitMineral();
            drivetrain.driveToPos(0.5,44,44,5);
        }
        else{
            drivetrain.driveToPos(0.5,17,17,5);
            if(vision.aligned()){
                hitMineral();
                drivetrain.driveToPos(0.5,27,27,5);
            }
            else {
                drivetrain.driveToPos(0.5,17,17,5);
                hitMineral();
                drivetrain.driveToPos(0.5,10,10,5);
            }
        }
    }


    public void markerClose(){
        drivetrain.turn(45,3);
        drivetrain.driveToPos(0.5,-72,-72,5);
        arm.armDown();
    }

    public void markerFar(){
        drivetrain.turn(45,3);
        drivetrain.driveToPos(0.5,72,72,5);
        arm.armDown();
    }

    public void parkClose(){
        drivetrain.driveToPos(0.75,-120,-120,10);
    }

    public void parkFar(){
        drivetrain.driveToPos(0.75,120,120,10);
    }

    public void turn(){
        drivetrain.turn(90,10);
    }

    public void forward(){drivetrain.driveToPos(0.5,24,24,10);}

    public void sampleTest(){
        if (!vision.aligned()) {
            drivetrain.driveToPos(0.3, 17, 17, 10);
            if (!vision.aligned()) {
                drivetrain.driveToPos(0.3, 17, 17, 10);
            }
        }
        drivetrain.driveToPos(0.3,5,5,10);
    }
}

