package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.hardware.Mechanism;


public class Lift extends Mechanism {
    private static final double     COUNTS_PER_MOTOR_REV    = 723.24;
    private static final double     SPROCKET_DIAMETER_INCHES   = 4.0;
    private static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV) / (SPROCKET_DIAMETER_INCHES * 3.1415);
    private static final double distance = 4;
    private static final double hangHeight = distance*COUNTS_PER_INCH;
    private static final double liftSpeed = 0.5;
    private DcMotor liftRight;
    private DcMotor liftLeft;
    private DistanceSensor sensorRange;
    private Servo hook;
    Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

    public void init(HardwareMap hwMap) {
        liftLeft = hwMap.dcMotor.get("liftLeft");
        liftRight = hwMap.dcMotor.get("liftRight");
        hook = hwMap.servo.get("hook");
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");

        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void upLift(){
        liftLeft.setTargetPosition((int)(liftLeft.getCurrentPosition()+hangHeight));
        liftRight.setTargetPosition((int)(liftRight.getCurrentPosition()+hangHeight));
        liftRight.setPower(liftSpeed);
        liftLeft.setPower(liftSpeed);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void downLift(){
        liftLeft.setTargetPosition((int)(liftLeft.getCurrentPosition()-hangHeight));
        liftRight.setTargetPosition((int)(liftRight.getCurrentPosition()-hangHeight));
        liftRight.setPower(liftSpeed);
        liftLeft.setPower(liftSpeed);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}