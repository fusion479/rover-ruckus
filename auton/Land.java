package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Timed Land", group = "test")
public class Land extends LinearOpMode{
    public HardwareMain robot = new HardwareMain(this);
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");
            telemetry.update();
        }
        robot.lift.unlock();
        robot.lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(2000);
        robot.lift.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.liftRight.setPower(0.2);
        robot.lift.liftLeft.setPower(0.2);
        sleep(700);
        robot.lift.liftRight.setPower(0);
        robot.lift.liftLeft.setPower(0);
        robot.lift.hook();
        sleep(1000);
        robot.drivetrain.driveToPos(0.8,80);
        robot.marker.armUp();
    }
}
