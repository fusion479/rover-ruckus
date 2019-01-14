package org.firstinspires.ftc.teamcode.RoverRuckus.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Mechanism;

public class TeamMarker extends Mechanism {

    private static final double upPosition = 0;
    private static final double downPosition = 0;
    private Servo arm;
    @Override
    public void init(HardwareMap hwMap) {
        arm = hwMap.get(Servo.class, "markerArm");
    }
    public void armUp(){
        arm.setPosition(upPosition);
    }
    public void armDown(){
        arm.setPosition(downPosition);
    }
}
