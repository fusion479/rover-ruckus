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
public class HardwareMain {

    /* Mechanisms */
    public Drivetrain drivetrain;
    public Vision vision;
    public SampleArm arm;
    public  Lift lift;
    public LinearOpMode opMode;

    public HardwareMain(LinearOpMode opMode){
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

    public void teleOpDrive(double drive, double rotate, double strafe){
        double leftPower = drive + rotate;
        double rightPower = drive  - rotate;
        drivetrain.setLeftPower(leftPower);
        drivetrain.setRightPower(rightPower);
        drivetrain.strafe(strafe);
    }
    public void land(){
        lift.liftToPos(-1,opMode);
        arm.armUp();
        lift.liftToPos(7, opMode);
        drivetrain.strafeToPos(0.5,2,3);
    }

    public void landJank(){
        lift.setLiftPower(-1);
        lift.unhook();
        opMode.sleep(500);
        lift.setLiftPower(0.5);
        opMode.sleep(1000);
        lift.setLiftPower(0);
        drivetrain.strafeToPos(1,5,10);
    }

    public void driveToSample(){
        drivetrain.driveToPos(0.5,
                Constants.landerToSample-Constants.robotLength, 5);
        drivetrain.strafeToPos(0.5,15,5);
    }

    public void sampleStrafe(){
        if (vision.aligned()){
            drivetrain.driveToPos(0.5,2,3);
            drivetrain.driveToPos(0.5,-2,3);
            drivetrain.strafeToPos(0.5,-34,5);
        }
        else {
            drivetrain.strafeToPos(0.5, -17, 5);
            if (vision.aligned()) {
                drivetrain.driveToPos(0.5, 2, 3);
                drivetrain.driveToPos(0.5, -2, 3);
                drivetrain.strafeToPos(0.5, -17, 5);
            } else {
                drivetrain.strafeToPos(0.5, -17, 5);
                if (vision.aligned()) {
                    drivetrain.driveToPos(0.5, 2, 3);
                    drivetrain.driveToPos(0.5, -2, 3);
                }
            }
        }
        drivetrain.strafeToPos(0.5,9,5);
    }


    public void markerCloseCorner(){
        drivetrain.turn(135,3);
        drivetrain.driveToPos(0.5,-72,5);
        arm.armDown();
    }

    public void markerFarCorner(){
        drivetrain.turn(135,3);
        drivetrain.driveToPos(0.5,72,5);
        arm.armDown();
    }

    public void parkClose(){
        drivetrain.driveToPos(0.75,-120,10);
    }

    public void parkFar(){
        drivetrain.driveToPos(0.75,120,10);
    }

    public void turn(){
        drivetrain.turn(90,10);
    }

    public void forward(){drivetrain.driveToPos(0.5,24,10);}

    public void sampleTest(){
        if (!vision.aligned()) {
            drivetrain.driveToPos(0.3,  17, 10);
            if (!vision.aligned()) {
                drivetrain.driveToPos(0.3,  17, 10);
            }
        }
        drivetrain.driveToPos(0.3,5,10);
    }

    public void strafe(){
        drivetrain.strafe(0.25);
    }
}

