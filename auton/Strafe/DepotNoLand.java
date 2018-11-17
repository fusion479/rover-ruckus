package org.firstinspires.ftc.teamcode.RoverRuckus.auton.Strafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Depot No Land", group = "HDrive: No Land")
public class DepotNoLand extends LinearOpMode {
    private HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.driveToSample();
        robot.sampleStrafe();
        robot.markerCloseCorner();
        robot.parkFar();
    }
}
