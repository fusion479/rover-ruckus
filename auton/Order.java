package org.firstinspires.ftc.teamcode.RoverRuckus.auton;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CV", group = "test")
public class Order extends LinearOpMode {
    public org.firstinspires.ftc.teamcode.hardware.HardwareMain robot = new org.firstinspires.ftc.teamcode.hardware.HardwareMain(this);
    public String position = "UNKNOWN";

    public void runOpMode() {
        robot.init(hardwareMap);
        while (!opModeIsActive() && !isStopRequested()) {
            if (position.equals("UNKNOWN")) {
                position = robot.vision.getOrder();
            }
            telemetry.addData("Gold Position", position);
            telemetry.update();
        }
        robot.drivetrain.turn(90,0.8);
    }
}