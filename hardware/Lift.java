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


public class Lift extends Mechanism {
    private static final double     COUNTS_PER_MOTOR_REV    = 1120;
    private static final double     SPROCKET_DIAMETER_INCHES   = 2.0;
    private static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV) / (SPROCKET_DIAMETER_INCHES * 3.1415);
    private static final double liftSpeed = 0.5;
    public DcMotor liftRight;
    public DcMotor liftLeft;
    private DistanceSensor sensorRange;
    private Servo hook;
    private boolean hasSensor;
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

    public void init(HardwareMap hwMap) {
//        if (sensor){
//            hasSensor = sensor;
//            sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
//        }
        liftLeft = hwMap.dcMotor.get("liftLeft");
        liftRight = hwMap.dcMotor.get("liftRight");
        hook = hwMap.servo.get("arm");
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setPower(0);
        liftRight.setPower(0);

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHook(double position){
        hook.setPosition(position);
    }

    public void hook(){
        setHook(0);
    }

    public  void unhook(){
        setHook(0.5);
    }

    public void setLiftPower(double power){
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    public void liftToPos(double distance, LinearOpMode opMode){
        opMode.telemetry.addData("encoder", liftRight.getCurrentPosition());
        opMode.telemetry.update();
        opMode.sleep(2000);
        liftRight.setTargetPosition((int)(liftRight.getCurrentPosition()+distance*COUNTS_PER_INCH));
        liftLeft.setTargetPosition((int)(liftLeft.getCurrentPosition()+distance*COUNTS_PER_INCH));
        opMode.telemetry.addData("encoder", liftRight.getCurrentPosition());
        opMode.telemetry.update();
        opMode.sleep(2000);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void land(){
//        if (hasSensor){
//            while (sensorRange.getDistance(DistanceUnit.INCH)> Constants.bottomOfRobotToGround){
//                liftLeft.setPower(liftSpeed);
//                liftRight.setPower(liftSpeed);
//            }
//            liftRight.setPower(0);
//            liftLeft.setPower(0);
//        }
//        else {
            liftLeft.setTargetPosition((int)(Constants.heightNeededToLift * COUNTS_PER_INCH));
            liftRight.setTargetPosition((int)(Constants.heightNeededToLift * COUNTS_PER_INCH));
            liftRight.setPower(liftSpeed);
            liftLeft.setPower(liftSpeed);
            liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
    }
    public void downLift(){
        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        liftRight.setPower(liftSpeed);
        liftLeft.setPower(liftSpeed);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}