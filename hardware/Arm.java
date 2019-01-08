package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Arm extends org.firstinspires.ftc.teamcode.hardware.Mechanism {
    private static final double armSpeed = 0.5;
    public DcMotor armRight;
    public DcMotor armLeft;
    public void init(HardwareMap hwMap) {
        armLeft = hwMap.dcMotor.get("armLeft");
        armRight = hwMap.dcMotor.get("armRight");
        armRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
        armLeft.setPower(1.0);
        armRight.setPower(1.0);
        armRight.setTargetPosition((int)(distance+armRight.getCurrentPosition()));
        armLeft.setTargetPosition((int)(distance+armLeft.getCurrentPosition()));
       armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
