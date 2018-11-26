package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@TeleOp(name = "Pikachu Detective", group = "HDrive")
public class HDriveWithLiftAndMarkerArm extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("liftLeft", robot.lift.liftLeft.getCurrentPosition());
            telemetry.addData("liftRight", robot.lift.liftRight.getCurrentPosition());
            telemetry.addData("leftFront", robot.drivetrain.leftFront.getCurrentPosition());
            telemetry.addData("leftBack", robot.drivetrain.leftBack.getCurrentPosition());
            telemetry.addData("rightFront", robot.drivetrain.rightFront.getCurrentPosition());
            telemetry.addData("rightBack", robot.drivetrain.rightBack.getCurrentPosition());
            telemetry.update();
            robot.teleOpDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
            if (gamepad1.dpad_up) {
                robot.lift.setLiftPower(0.4);
            } else {
                if (gamepad1.dpad_down) {
                    robot.lift.setLiftPower(-1);
                } else {
                    robot.lift.setLiftPower(0);
                }
            }
            if (gamepad1.x || gamepad2.x) {
                robot.arm.armUp();
            }
            if (gamepad1.a || gamepad2.a) {
                robot.arm.armDown();
            }
        }
        robot.lift.setLiftPower(gamepad2.right_trigger);
        robot.lift.setLiftPower(gamepad2.left_trigger);
        if (gamepad1.b || gamepad2.b){
//            robot.lift.hook();
        }
        if (gamepad1.y || gamepad2.y){
//            robot.lift.unhook();
        }
    }
}
