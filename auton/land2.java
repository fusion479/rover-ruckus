package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Encoder Land", group = "test")
public class land2 extends LinearOpMode{
    public HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Gold Position", robot.vision.getOrder());
            telemetry.update();
        }
        robot.lift.unlock();
        robot.lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lift.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.land();
        telemetry.addData("left motor", robot.lift.liftLeft.getCurrentPosition());
        telemetry.addData("right motor", robot.lift.liftRight.getCurrentPosition());
        telemetry.update();
        robot.lift.hook();
    }
}

