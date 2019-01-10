package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Land and Sample", group = "Test")
public class LandAndSample extends LinearOpMode {
    private  HardwareMain robot = new HardwareMain(this);
    public void runOpMode ()throws InterruptedException {
        String sampleResult;
        robot.init(hardwareMap);
        robot.vision.sampleInit(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        sampleResult = robot.vision.getOrder();
        robot.vision.stopOrder();
        robot.vision.goldAlignInit(hardwareMap);
        robot.lift.land();
        robot.driveToSample();
        while(!robot.vision.aligned()){
            robot.drivetrain.setLeftPower(0.2);
            robot.drivetrain.setRightPower(0.2);
        }
        robot.drivetrain.setLeftPower(0);
        robot.drivetrain.setRightPower(0);
        robot.drivetrain.turn(90,0.2);
        robot.markerFarCorner();
        robot.parkClose();
    }
}
