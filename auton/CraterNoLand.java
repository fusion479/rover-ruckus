package org.firstinspires.ftc.teamcode.RoverRuckus.auton;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Crater No Land", group = "Red")
public class CraterNoLand extends LinearOpMode {
    private HardwareMain robot = new HardwareMain();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.driveToSample();
        robot.sampleStrafe();
        robot.markerFarCorner();
        robot.park();
    }
}
