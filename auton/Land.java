package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Gravity Land Marker", group = "test")
public class Land extends LinearOpMode{
    public HardwareMain robot = new HardwareMain(this);
    public String position = "UNKNOWN";
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            if (position.equals("UNKNOWN")){
                position = robot.vision.getOrder();
            }
            telemetry.addData("Gold Position", position);
            telemetry.update();
        }
        robot.lift.unlock();
        robot.lift.liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.lift.liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(500);
        robot.lift.setHook(1);
        sleep(1000);
        if (position.equals("RIGHT")) {
            robot.drivetrain.turn(20, 0.5); //right
            robot.drivetrain.driveToPos(0.8, 40);
            robot.drivetrain.turn(115, 0.5);
            robot.drivetrain.driveToPos(0.8, -20);
            robot.marker.armUp();
            sleep(500);
//            robot.drivetrain.turn(-90, 0.5);
            robot.drivetrain.driveToPos(0.8, 120);
        }
        else{
            if (position.equals("LEFT")) {
                robot.drivetrain.turn(-25, 0.5);
                robot.drivetrain.driveToPos(0.8, 40);
                robot.drivetrain.turn(70, 0.5);
                robot.drivetrain.driveToPos(0.8, 20);
                robot.marker.armUp();
                sleep(500);
//                robot.drivetrain.turn(-90, 0.5);
                robot.drivetrain.driveToPos(0.8, -120);
            }
            else{
                robot.drivetrain.driveToPos(0.8, 40);
                robot.drivetrain.turn(45, 0.5);
                robot.marker.armUp();
                sleep(500);
//                robot.drivetrain.turn(-90, 0.5);
                robot.drivetrain.driveToPos(0.8, -120);
            }
        }

    }
}
