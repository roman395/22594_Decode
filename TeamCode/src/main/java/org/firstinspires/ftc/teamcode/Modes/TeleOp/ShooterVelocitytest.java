package org.firstinspires.ftc.teamcode.Modes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;

@Configurable
@TeleOp
public class ShooterVelocitytest extends LinearOpMode {
    public static double velocity = 0;
    public static double pos = 0, power = 0;
    public static int id = 24;
    public static boolean isCameraUse = false;
    public static boolean reinitializeShooterNew = false;
    private boolean reinitializeShooterOld = false;
    public static long expose = 4;
    ExposureControl control;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        Shooter shooter = new Shooter(this, id);
        Turret turret = new Turret(this);
        Mecanum mecanum = new Mecanum(this);
        waitForStart();
        while (opModeIsActive()) {
            mecanum.teleOp();
            if(reinitializeShooterNew!=reinitializeShooterOld)
            {
                shooter = new Shooter(this, id);
                reinitializeShooterOld = reinitializeShooterNew;
            }
            shooter.advancedTelemetry();
            shooter.testing(velocity, pos);
            turret.AutoAimingOnError(shooter.getBearing());
            shooter.intakeModule.setFeederPower(-power);
            shooter.intakeModule.setIntakePower(power);
        }
    }
    /*
    dist velocity pose
    1) 1.47 1700 1
    2) 2.16 1900 1
    3) 1.89 1800 1
    3) 0.88 1650 0.75
    3) 3.51 2200 1

     */
}
