package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@TeleOp(name = "Test", group = "Test")
public class Everything extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    public boolean hooked = false;
    public boolean x = false;
    public boolean y = false;
    public boolean locked = false;
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.vision.stopGoldAlign();
        waitForStart();
        while(opModeIsActive()) {
            //robot.acquirer.sendTelemetry();
            //robot.drivetrain.sendTelemetry();
            //robot.lift.sendTelemetry();
//            telemetry.update();
            robot.teleOpDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x);

            if (gamepad1.a || gamepad2.a) {
                if(hooked){
                    robot.lift.unhook();
                    hooked =!hooked;
                }
                else{
                    robot.lift.hook();
                    hooked =!hooked;
                }
            }
            if (gamepad1.x) {
                if(x){
                    robot.acquirer.acquirerForward();
                    x=!x;
                }
                else{
                    robot.acquirer.acquirerOff();
                    x=!x;
                }
            }
            if (gamepad1.y) {
                if(y){
                    robot.acquirer.armUp();
                    y=!y;
                }
                else{
                    robot.acquirer.returnArm();
                    y=!y;
                }
            }

            if(gamepad1.b || gamepad2.b){
                if(locked){
                    robot.lift.unlock();
                    locked = !locked;
                }
                else{
                    robot.lift.lock();
                    locked = !locked;
                }
            }
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                robot.lift.setLiftPower(-1);
            } else {
                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    robot.lift.setLiftPower(0.3);
                } else {
                    robot.lift.setLiftPower(0);
                }
            }
            robot.acquirer.setArmPower(-gamepad2.right_stick_y);
        }
        if(gamepad2.x){
            robot.acquirer.acquirerForward();
        }
        else{
            robot.acquirer.acquirerOff();
        }
    }
}
