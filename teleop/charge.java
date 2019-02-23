package org.firstinspires.ftc.teamcode.RoverRuckus.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp(name = "charge", group = "charge")
public class charge extends LinearOpMode {
    public void runOpMode(){
        RevExtensions2.init();
        ExpansionHubEx expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");;
        expansionHub.setPhoneChargeEnabled(true);
        waitForStart();
        while(opModeIsActive()){
        }
    }
}
