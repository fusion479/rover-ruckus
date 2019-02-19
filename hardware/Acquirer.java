package org.firstinspires.ftc.teamcode.RoverRuckus.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevExtensions2;

public class Acquirer extends Mechanism {

    public DcMotor armRight;
    public DcMotor armLeft;
    public ExpansionHubServo acquireLeft;
    public ExpansionHubServo acquireRight;
    public LinearOpMode opMode;

    public Acquirer(LinearOpMode opMode){
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        RevExtensions2.init();

        armLeft = hwMap.dcMotor.get("armLeft");
        armRight = hwMap.dcMotor.get("armRight");
        acquireLeft= (ExpansionHubServo) hwMap.servo.get("acquireLeft");
        acquireRight= (ExpansionHubServo) hwMap.servo.get("acquireRight");
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

    public void setArmPower(double power){
        armLeft.setPower(power);
        armRight.setPower(power);
    }

    public void returnArm(){
        armRight.setTargetPosition(0);
        armLeft.setTargetPosition(0);
        setArmPower(1);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void armUp(){
        armRight.setTargetPosition(500);
        armLeft.setTargetPosition(500);
        setArmPower(1);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void acquirerForward(){
        acquireLeft.setPulseWidthUs(1000);
        acquireRight.setPulseWidthUs(2000);
    }

    public void acquirerOff(){
        acquireLeft.setPulseWidthUs(1500);
        acquireRight.setPulseWidthUs(1500);
    }

    public void acquirerReverse() {
        acquireLeft.setPulseWidthUs(2000);
        acquireRight.setPulseWidthUs(1000);
    }

    public void sendTelemetry(){
        opMode.telemetry.addData("Left Arm", armLeft.getCurrentPosition());
        opMode.telemetry.addData("Right Arm", armRight.getCurrentPosition());
    }
}

