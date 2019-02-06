import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hardware.HardwareMain;

@Autonomous(name = "Middle Marker Floor", group = "test")
public class Middle extends LinearOpMode {
    public  HardwareMain robot = new HardwareMain(this);
    public String position = "UNKNOWN";
    public void runOpMode(){
        robot.init(hardwareMap);
        while (!opModeIsActive()&&!isStopRequested()) {
            if (position.equals("UNKNOWN")){
                position = robot.vision.getOrder();
            }
            telemetry.addData("Gold Position", position);
            telemetry.update();
        }
        robot.drivetrain.driveToPos(0.8, 40);
        robot.drivetrain.turn(45, 0.5);
        robot.marker.armUp();
        sleep(500);
        robot.drivetrain.driveToPos(0.8, -120);
    }
}
