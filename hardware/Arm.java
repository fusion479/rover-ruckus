package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoverRuckus.Constants;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;


public class Arm extends Mechanism {
    private static final double armSpeed = 0.5;
    public DcMotor armRight;
    public DcMotor armLeft;

    public void init(HardwareMap hwMap) {
        armLeft = hwMap.dcMotor.get("armLeft");
        armRight = hwMap.dcMotor.get("armRight");
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLeft.setPower(0);
        armRight.setPower(0);

        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armTarget(double distance){
        opMode.telemetry.addData("encoder", armRight.getCurrentPosition());
        opMode.telemetry.update();
        opMode.sleep(2000);
//        armRight.setTargetPosition(distance);
//        armLeft.setTargetPosition(distance);
//       armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void armOrigin(){
        armLeft.setTargetPosition(0);
        armRight.setTargetPosition(0);
        armRight.setPower(armSpeed);
        armLeft.setPower(armSpeed);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
