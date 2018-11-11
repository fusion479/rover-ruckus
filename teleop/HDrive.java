package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.Drivetrain;

@TeleOp(name = "HDrive", group = "Test")
public class HDrive extends LinearOpMode {
    private Drivetrain drivetrain = new Drivetrain();
    private double leftPower;
    private double rightPower;
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        drivetrain.encoderInit();
        while(opModeIsActive()){
            leftPower = gamepad1.right_stick_y - gamepad1.right_stick_y;
            rightPower = gamepad1.right_stick_y + gamepad1.right_stick_y;
            drivetrain.setLeftPower(leftPower);
            drivetrain.setRightPower(rightPower);
            drivetrain.strafe(gamepad1.left_stick_x);
        }
    }
}
