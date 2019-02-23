package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name="tensor test", group = "test")
public class MarkerRisky extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.vision.initVuforia();
        robot.vision.initTfod(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            robot.vision.getTensorFlow();
            telemetry.addData("position", robot.vision.goldPos);
            telemetry.update();
        }
    }
}
