package org.firstinspires.ftc.teamcode.Modes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;

@TeleOp
public class TeleopBLUE extends LinearOpMode {
    Mecanum mecanum;
    Intake intake;
    Shooter shooter;
    Telemetry tel = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        mecanum = new Mecanum(this);
        intake = new Intake(this);
        shooter = new Shooter(this, tel);

        waitForStart();
        while (opModeIsActive()) {
            mecanum.TeleOp();
            shooter.TeleOp(intake, 20);
            telemetry.update();
        }
        shooter.getVisionPortal().stopStreaming();
        shooter.getVisionPortal().stopLiveView();
    }
}
