package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Depot Single Sample", group = "Red")
public class DepotEverything extends LinearOpMode {
    private  HardwareMain robot = new HardwareMain();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.driveToSample();
        robot.sampleStrafe();
        robot.markerCloseCorner();
        robot.park();
    }
}
