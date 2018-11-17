package org.firstinspires.ftc.teamcode.RoverRuckus.auton.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "turn90", group = "test")
public class Turn45 extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.turn();
    }
}
