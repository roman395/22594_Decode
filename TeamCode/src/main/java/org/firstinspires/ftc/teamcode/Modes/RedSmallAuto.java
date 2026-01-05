package org.firstinspires.ftc.teamcode.Modes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;

@Configurable
@Autonomous
public class RedSmallAuto extends LinearOpMode {
    Mecanum drive;
    Shooter shooter;
    Intake intake;
    public static double drivePower = 1;
    public static double strafePower = -1;
    public static double driveTime = 0.5;
    public static double strafeTime = 3;
    public static double shooterTime = 5;
    public static double intakePower = 0.6;
    public static double intakeWaitTime = 0.7;
    public static double intakeWorkTime = 3;
    public static int intakeCycles = 1;

    boolean isTrainComplete, firstTrainComplete = false;
    boolean isShooterComplete, firstShooterCompletes = false;
    Telemetry tel = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        driveTime *= 1000;
        shooterTime *= 1000;
        intakeWaitTime *= 1000;
        intakeWorkTime *= 1000;
        strafeTime *= 1000;
        drive = new Mecanum(this);
        shooter = new Shooter(this, telemetry);
        intake = new Intake(this);

        waitForStart();


        drive.ResetTimer();
        shooter.ResetTimer();

        while (opModeIsActive()) {
            shooter.Update();
            tel.addData("IsTrainComplete", isTrainComplete = drive.ForwardMove(driveTime, drivePower));
            tel.addData("time", drive.GetTimer().milliseconds());
            if (isTrainComplete && !firstTrainComplete && !isShooterComplete) {
                intake.ResetTimer();
                firstTrainComplete = true;
            }
            if (firstTrainComplete && !isShooterComplete) {
                intake.Autonom(intakePower, intakeWorkTime, intakeWaitTime, intakeCycles);
                isShooterComplete = shooter.Autonom(shooterTime, 24 );
                if (isShooterComplete)
                    drive.ResetTimer();
            }
            if (isShooterComplete)
                drive.StrafeMove(strafeTime, 1 * strafePower, -1 * strafePower, -1 * strafePower, 1 * strafePower);
            tel.addData("is shoot complite", isShooterComplete);
            tel.update();
        }
        shooter.getVisionPortal().stopStreaming();
        shooter.getVisionPortal().stopLiveView();
    }


}
