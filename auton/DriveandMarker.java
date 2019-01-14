package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "drive and marker", group = "marker")
public class DriveandMarker extends LinearOpMode {
    public HardwareMain robot;
    public void runOpMode(){
        robot = new HardwareMain(this);
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        robot.drivetrain.driveToPos(0.8,80);
        robot.marker.armUp();
    }
}
