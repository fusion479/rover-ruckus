package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoverRuckus.Constants;
import org.firstinspires.ftc.teamcode.RoverRuckus.Vision;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.Acquirer;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.TeamMarker;
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
    public  Lift lift;
    public LinearOpMode opMode;
    public Acquirer acquirer;
    public TeamMarker marker;

    public HardwareMain(LinearOpMode opMode){
        drivetrain = new Drivetrain(opMode);
        vision = new Vision();
        this.opMode = opMode;
        lift = new Lift(opMode);
        acquirer = new Acquirer(opMode);
        marker = new TeamMarker();
    }
    /**
     * Initializes all mechanisms on the robot.
     * @param hwMap     robot's hardware map
     */
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        drivetrain.encoderInit();
        vision.goldAlignInit(hwMap);
        lift.init(hwMap);
        acquirer.init(hwMap);
        marker.init(hwMap);
    }

    public void teleOpDrive(double drive, double rotate){
        double leftPower = drive + rotate;
        double rightPower = drive  - rotate;
        drivetrain.setLeftPower(leftPower);
        drivetrain.setRightPower(rightPower);
    }
    public void driveToSample(){
        drivetrain.driveToPos(0.5,
                Constants.landerToSample-Constants.robotLength);
        drivetrain.turn(-90,0.3);
        drivetrain.driveToPos(0.5,17);
    }


    public void markerCloseCorner(){
        drivetrain.turn(135,0.5);
        drivetrain.driveToPos(0.5,-72);
    }

    public void markerFarCorner(){
        drivetrain.turn(135,0.5);
        drivetrain.driveToPos(0.5,72);
    }

    public void parkClose(){
        drivetrain.driveToPos(0.75,-120);
    }

    public void parkFar(){
        drivetrain.driveToPos(0.75,120);
    }

    public void turn(){
        drivetrain.turn(90,0.5);
    }

    public void forward(){drivetrain.driveToPos(0.5,24);}

    public void sampleTest(){
        if (!vision.aligned()) {
            drivetrain.driveToPos(0.3,  -17);
            if (!vision.aligned()) {
                drivetrain.driveToPos(0.3,  -17);
            }
        }
        drivetrain.driveToPos(0.3,5);
    }
}

