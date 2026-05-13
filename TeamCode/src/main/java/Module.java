import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Module {
    public Module(LinearOpMode linearOpMode){}
    public void update(){}
    public void addTelemetry(Telemetry telemetry){}
}
