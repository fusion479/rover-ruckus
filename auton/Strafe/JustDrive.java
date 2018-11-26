package org.firstinspires.ftc.teamcode.RoverRuckus.auton.Strafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareTank;

@Autonomous (name = "CraterDrive", group = "crate")
public class JustDrive extends LinearOpMode {
    public HardwareTank robot = new HardwareTank(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) { telemetry.addData("Status", "Waiting in Init"); telemetry.update(); }
//        waitForStart();
        robot.drivetrain.driveToPos(0.5,80,5);
    }
}
