package org.firstinspires.ftc.teamcode.RoverRuckus.auton.NoStrafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareTank;

@Autonomous(name = "Crater without Land (Tank)", group = "Tank: No Land")
public class CraterNoLand extends LinearOpMode {
    public HardwareTank robot = new HardwareTank(this);
    public int a =0;
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        telemetry.addData("Step", a);
        telemetry.update();
        a++;
        robot.driveToSample();
        telemetry.addData("Step", a);
        telemetry.update();
        a++;
        robot.sample();
        telemetry.addData("Step", a);
        telemetry.update();
        a++;
        robot.markerFar();
        telemetry.addData("Step", a);
        telemetry.update();
        a++;
        robot.parkClose();
        telemetry.addData("Step", a);
        telemetry.update();
        a++;
    }
}