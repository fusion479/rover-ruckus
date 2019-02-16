package org.firstinspires.ftc.teamcode.RoverRuckus.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Encoder Marker", group = "Everything")
public class Encoder extends LinearOpMode{
    public HardwareMain robot = new HardwareMain(this);
    public String position = "UNKNOWN";
    public String temp;
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            temp = robot.vision.getOrder();
            if(temp.equals("RIGHT")||temp.equals("LEFT")){
                position = temp;
            }
            telemetry.addData("Gold Position", position);
            telemetry.update();
        }
        robot.vision.stopOrder();
        robot.lift.unlock();
        sleep(1000);
        robot.lift.land();
        sleep(500);
        robot.lift.setHook(0);
        sleep(1500);
        if (position.equals("LEFT")) {
            telemetry.addData("Path", "right");
            telemetry.update();
            robot.drivetrain.driveToPos(0.8,4);
            robot.drivetrain.turn(30, 0.5); //right
            robot.drivetrain.driveToPos(0.8, 40);
            robot.drivetrain.turn(-70, 0.5);
            robot.drivetrain.driveToPos(0.8, 33);
            robot.marker.armUp();
            sleep(500);
            robot.drivetrain.turn(-90,0.5);
//            robot.drivetrain.turn(10,0.3);
            robot.drivetrain.driveToPos(-0.8, 80);
        }
        else{
            if (position.equals("RIGHT")) {
                telemetry.addData("Path", "left");
                telemetry.update();
                robot.drivetrain.driveToPos(0.8,4);
                robot.drivetrain.turn(-30, 0.5);
                robot.drivetrain.driveToPos(0.8, 40);
                robot.drivetrain.turn(70, 0.5);
                robot.drivetrain.driveToPos(0.8, 20);
                robot.drivetrain.turn(90,0.5);
                robot.drivetrain.turn(90,0.5);
                robot.drivetrain.driveToPos(0.8,-10);
                robot.marker.armUp();
                sleep(500);
                robot.drivetrain.driveToPos(-0.8, 90);
            }
            else{
                telemetry.addData("Path", "middle");
                robot.drivetrain.driveToPos(0.8, 70);
                robot.drivetrain.turn(-90, 0.5);
                sleep(500);
                robot.marker.armUp();
                sleep(500);
                robot.drivetrain.turn(-45, 0.5);
                robot.drivetrain.driveToPos(0.8, 10);
//                robot.drivetrain.turn(10,0.3);
                robot.drivetrain.driveToPos(-0.8, 90);
            }
        }
        robot.lift.liftDown();
    }
}

