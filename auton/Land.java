package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Land", group = "test")
public class Land extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    public void runOpMode()throws InterruptedException {
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
            robot.lift.land();
            robot.drivetrain.driveToPos(0.8,80);
    }
}
