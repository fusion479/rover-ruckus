package org.firstinspires.ftc.teamcode.RoverRuckus.auton.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareTank;

@Autonomous(name = "arm", group= "arm")
public class marker extends LinearOpMode {
    public HardwareTank robot = new HardwareTank(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.arm.armUp();
    }
}
