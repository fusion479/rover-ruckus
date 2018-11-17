package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@TeleOp(name = "LiftTest", group = "test")
public class LiftTest extends LinearOpMode {
    public Lift lift = new Lift();
    public void runOpMode(){
        lift.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.dpad_up){
                lift.setLiftPower(0.2);
            }
            else{
                if (gamepad1.dpad_down){
                    lift.setLiftPower(-0.2);
                }
                else{
                    lift.setLiftPower(0);
                }
            }
        }
    }
}
