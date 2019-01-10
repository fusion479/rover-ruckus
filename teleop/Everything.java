package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@TeleOp(name = "Test", group = "Test")
public class Everything extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        while(opModeIsActive()) {
            robot.acquirer.sendTelemetry();
            robot.drivetrain.sendTelemetry();
            robot.lift.sendTelemetry();
            telemetry.update();
            robot.teleOpDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x);
            if (gamepad1.a) {
                robot.acquirer.acquirerForward();
            }
            if (gamepad1.x) {
                robot.acquirer.acquirerReverse();
            }
            if (gamepad1.y) {
                robot.acquirer.acquirerOff();
            }
            if (gamepad1.dpad_up) {
                robot.lift.setLiftPower(0.3);
            } else {
                if (gamepad1.dpad_down) {
                    robot.lift.setLiftPower(-0.3);
                } else {
                    robot.lift.setLiftPower(0);
                }
            }
            robot.acquirer.setArmPower(gamepad1.right_trigger);
            robot.acquirer.setArmPower(gamepad1.left_trigger);
        }
    }
}
