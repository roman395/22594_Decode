package org.firstinspires.ftc.teamcode.Deprecated.Modes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Turret;

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
        Shooter shooter = new Shooter(this, id, new Pose(10,10));
        Turret turret = new Turret(this, new Pose(10,10));
        Mecanum mecanum = new Mecanum(this);
        mecanum.loadStartPose();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            mecanum.teleOp();
            if(reinitializeShooterNew!=reinitializeShooterOld)
            {
                shooter = new Shooter(this, id, new Pose(10,10));
                reinitializeShooterOld = reinitializeShooterNew;
            }
            shooter.advancedTelemetry(telemetry);
            shooter.testing(velocity, pos);
            //turret.aimingOnOdo(new Pose(135,135, Math.toRadians(90)),mecanum.getPose());
            shooter.intakeModule.setFeederPower(-power);
            shooter.intakeModule.setIntakePower(power);
            telemetry.update();
        }
        GlobalStorage.lastPose = mecanum.getPose();

    }
    /*
    dist velocity pose
    1) 0.74 1400 0.2
    2) 0.98 1500 0.3
    3) 1.32 1600 0.4
    4) 1.71 1700 0.5
    5) 1.95 1800 0.6
    6) 2.33 1900 0.7
    7) 2.66 2000 0.8
    8) 3.15 2100 0.9


    1) 0.94 1500 0.3
    2) 1.3 1600 1
    3) 1.54 1700 1
    3) 1.75 1750 1
    4) 2.29 1900 1

     */
}
