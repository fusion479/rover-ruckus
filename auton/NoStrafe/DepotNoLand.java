package org.firstinspires.ftc.teamcode.RoverRuckus.auton.NoStrafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareTank;

@Autonomous(name = "Depot without Land (Tank)", group = "Tank: No Land")
public class DepotNoLand extends LinearOpMode {
    public HardwareTank robot = new HardwareTank(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.driveToSample();
        robot.sample();
        robot.markerClose();
        robot.parkFar();
    }
}
