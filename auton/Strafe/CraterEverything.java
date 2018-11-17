package org.firstinspires.ftc.teamcode.RoverRuckus.auton.Strafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Crater Everything (HDrive)", group = "HDrive: Everything")
public class CraterEverything extends LinearOpMode {
    private HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.land();
        robot.driveToSample();
        robot.sampleStrafe();
        robot.markerFarCorner();
        robot.parkClose();
    }
}
