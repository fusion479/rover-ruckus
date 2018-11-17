package org.firstinspires.ftc.teamcode.RoverRuckus.auton.Strafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Depot Everything (HDrive)", group = "HDrive: Everything")
public class DepotEverything extends LinearOpMode {
    private  HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.land();
        robot.driveToSample();
        robot.sampleStrafe();
        robot.markerCloseCorner();
        robot.parkFar();
    }
}
