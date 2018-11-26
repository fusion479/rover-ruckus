package org.firstinspires.ftc.teamcode.RoverRuckus.auton.Strafe;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "LandJank", group = "jank")
public class land extends LinearOpMode {
    public  HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.lift.unhook();
        robot.landJank();
        robot.drivetrain.strafeToPos(0.3,10,10);
        robot.drivetrain.driveToPos(0.5,80,5);
    }
}
