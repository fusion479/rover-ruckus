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
    public boolean acquire = false;
    public boolean expel = false;
    public void runOpMode(){
        robot.teleOpInit(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            robot.acquirer.sendTelemetry();
            robot.drivetrain.sendTelemetry();
            robot.lift.sendTelemetry();
            telemetry.update();
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
            if (gamepad2.right_trigger>0){
                robot.acquirer.acquirerForward();
            }
            else{
                if (gamepad2.left_trigger>0){
                    robot.acquirer.acquirerReverse();
                }
                else{
                    robot.acquirer.acquirerOff();
                }
            }
            robot.acquirer.setArmPower(-gamepad2.right_stick_y);

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
            if (gamepad1.dpad_down || gamepad2.left_bumper) {
                robot.lift.setLiftPower(-1);
            } else {
                if (gamepad1.dpad_up || gamepad2.right_bumper) {
                    robot.lift.setLiftPower(0.3);
                } else {
                    robot.lift.setLiftPower(0);
                }
            }
        }
    }
}
