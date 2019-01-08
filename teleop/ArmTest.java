package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.Drivetrain;

@TeleOp(name = "ArmTest", group = "Test")
public class ArmTest extends LinearOpMode {
    private Drivetrain drivetrain = new Drivetrain();
//    private Lift lift = new Lift();
    private org.firstinspires.ftc.teamcode.hardware.Arm acquireArm = new org.firstinspires.ftc.teamcode.hardware.Arm();
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        drivetrain.encoderInit();
        acquireArm.init(hardwareMap);
        double leftPower;
        double rightPower;
        double drive;
        double rotate;
        waitForStart();
        while(opModeIsActive()){
            drive = gamepad1.left_stick_y;
            rotate = -gamepad1.right_stick_x;
            leftPower = drive + rotate;
            rightPower = drive  - rotate;
            drivetrain.setLeftPower(leftPower);
            drivetrain.setRightPower(rightPower);
            telemetry.addData("encoder Right", acquireArm.armRight.getCurrentPosition());
            telemetry.addData("encoder Left", acquireArm.armRight.getCurrentPosition());
            telemetry.update();
            if (gamepad1.b){
              acquireArm.armTarget(10.0);
                sleep(2000);
            }
            if(gamepad1.a){
                acquireArm.armOrigin();
            }
        }
    }
}
