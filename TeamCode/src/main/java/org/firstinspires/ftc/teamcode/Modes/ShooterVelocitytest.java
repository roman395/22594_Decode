package org.firstinspires.ftc.teamcode.Modes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter;

@Configurable
@TeleOp
public class ShooterVelocitytest extends LinearOpMode {
    public static double velocity = 0;
    public static double pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        Shooter sh = new Shooter(this);
        Intake intake = new Intake(this);
        waitForStart();
        sh.ResetPIDTimer();
        while (opModeIsActive()) {
            sh.VelocityTests(velocity, pos);
            intake.TeleOp();
            telemetry.update();
        }
    }
}
