package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.RoverRuckus.hardware.SampleArm;

@TeleOp(name = "HDrive", group = "Test")
public class HDrive extends LinearOpMode {
    private Drivetrain drivetrain = new Drivetrain();
//    private Lift lift = new Lift();
    private SampleArm arm = new SampleArm();
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        drivetrain.encoderInit();
        arm.init(hardwareMap);
//        lift.init(hardwareMap);
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
            if (gamepad1.x){
                arm.armDown();
            }
            if(gamepad1.a){
                arm.armUp();
            }
//            if (gamepad1.right_trigger>0){
//                lift.land();
//            }
//            if(gamepad1.right_trigger>0){
//                lift.downLift();
//            }
//            if (gamepad1.right_bumper){
//                lift.setLiftPower(0.2);
//            }
//            else {
//                if (gamepad1.left_bumper){
//                    lift.setLiftPower(-0.2);
//                }
//                else {
//                    lift.setLiftPower(0);
//                }
//            }
        }
    }
}
