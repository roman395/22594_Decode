package org.firstinspires.ftc.teamcode.Tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Mecanum;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;
@Configurable
@TeleOp
public class penis extends LinearOpMode {
    public static int aprilTag = 24;
    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(this, new Pose(0,0,0));
        Mecanum drive = new Mecanum(this);
        Shooter shooter = new Shooter(this, aprilTag, new Pose(0,0,0));

        waitForStart();
        while (opModeIsActive())
        {
            drive.teleOp();
            //turret.advancedTelemetry(telemetry);
            shooter.advancedTelemetry(telemetry);
            telemetry.update();
            turret.TeleOp();
            if(gamepad1.startWasPressed())
                turret.resetStartPose();
        }
    }
}
