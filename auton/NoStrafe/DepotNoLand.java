package org.firstinspires.ftc.teamcode.RoverRuckus.auton.NoStrafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareTank;

@Autonomous(name = "Depot without Land (Tank)", group = "Tank: No Land")
public class DepotNoLand extends LinearOpMode {
    public HardwareTank robot = new HardwareTank(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) { telemetry.addData("Status", "Waiting in Init"); telemetry.update(); }
//        waitForStart();
        robot.drivetrain.driveToPos(0.5,60,60,10);
        robot.arm.armDown();
        robot.drivetrain.turn2(45,10);
        robot.drivetrain.driveToPos(0.5,-140,-140, 10);
    }
}
