package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@TeleOp(name = "Test", group = "Test")
public class Everything extends LinearOpMode {
    public HardwareMain robot = new HardwareMain(this);
    public boolean hooked = false;
    public boolean xPast,yPast,bPast,aPast;
    public boolean xCurrent,yCurrent,bCurrent,aCurrent;
    public boolean locked = false;
    public void runOpMode(){
        robot.teleOpInit(hardwareMap);
        xPast = false;
        yPast = false;
        bPast = false;
        aPast = false;
        waitForStart();
        while(opModeIsActive()) {
            xCurrent = gamepad1.x || gamepad2.x;
            yCurrent = gamepad1.y || gamepad2.y;
            aCurrent = gamepad1.a || gamepad2.a;
            bCurrent = gamepad1.b || gamepad2.b;
            robot.acquirer.sendTelemetry();
            robot.drivetrain.sendTelemetry();
            robot.lift.sendTelemetry();
            telemetry.update();
            robot.teleOpDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x);

            if (aCurrent == true && aPast == false) {
                if(hooked){
                    robot.lift.setHook(0);
                    hooked =!hooked;
                }
                else{
                    robot.lift.hook();
                    hooked =!hooked;
                }
            }
            if (xCurrent == true && xPast == false) {
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
            robot.acquirer.setArmPower(-gamepad2.right_stick_y/2);

            if(bCurrent  && !bPast){
                if(locked){
                    robot.lift.unlock();
                    locked = !locked;
                }
                else{
                    robot.lift.lock();
                    locked = !locked;
                }
            }
            if(yCurrent && !yPast){
                robot.lift.land();
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
            xPast = xCurrent;
            aPast = aCurrent;
            bPast = bCurrent;
            yPast = yCurrent;
        }
    }
}
