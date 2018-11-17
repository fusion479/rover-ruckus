package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@TeleOp(name = "More HDrive", group = "HDrive")
public class HDriveWithLiftAndMarkerArm extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.teleOpDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.left_stick_x);
            if (gamepad1.dpad_up) {
                robot.lift.setLiftPower(0.2);
            } else {
                if (gamepad1.dpad_down) {
                    robot.lift.setLiftPower(-0.2);
                } else {
                    robot.lift.setLiftPower(0);
                }
            }
            if (gamepad1.x) {
                robot.arm.armUp();
            }
            if (gamepad1.a) {
                robot.arm.armDown();
            }
        }
        if(gamepad1.right_trigger>0){
            robot.lift.downLift();
        }
        if(gamepad1.left_trigger>0){
            robot.lift.land();
        }

    }
}
