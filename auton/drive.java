package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;
@Autonomous(name = "Turn 90", group = "test")
public class drive extends LinearOpMode {
    public org.firstinspires.ftc.teamcode.hardware.HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        robot.drivetrain.turn(90,0.5);
        telemetry.addData("current angle",robot.drivetrain.getAngle());
        telemetry.update();
    }
}
