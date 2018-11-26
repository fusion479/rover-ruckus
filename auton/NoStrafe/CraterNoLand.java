package org.firstinspires.ftc.teamcode.RoverRuckus.auton.NoStrafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareTank;

@Autonomous(name = "Crater without Land (Tank)", group = "Tank: No Land")
@Disabled
public class CraterNoLand extends LinearOpMode {
    public HardwareTank robot = new HardwareTank(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) { telemetry.addData("Status", "Waiting in Init"); telemetry.update(); }
//        waitForStart();
        robot.drivetrain.driveToPos(0.5,-68,10);
        robot.arm.armDown();
        robot.drivetrain.turn(45,10);
        robot.drivetrain.driveToPos(0.5,-140, 10);
    }
}