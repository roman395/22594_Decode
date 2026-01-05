package org.firstinspires.ftc.teamcode.Modes;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.concurrent.TimeUnit;

@Configurable
@TeleOp
public class ShooterVelocitytest extends LinearOpMode {
    public static double velocity = 0;
    public static double pos = 0, power = 0.6;
    public static int id = 24;
    public static boolean isCameraUse = false;
    public static long expose = 4;
    ExposureControl control;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();
        Shooter sh = new Shooter(this, telemetry);
        Intake intake = new Intake(this);
        sh.getVisionPortal().resumeStreaming();
        sh.getVisionPortal().resumeLiveView();
        PanelsCameraStream.INSTANCE.startStream(sh.getVisionPortal(), 30);
        control = sh.getVisionPortal().getCameraControl(ExposureControl.class);

        control.setMode(ExposureControl.Mode.Manual);
        waitForStart();


        while (opModeIsActive()) {
            control.setExposure(expose, TimeUnit.MILLISECONDS);
            sh.VelocityTests(velocity, pos, id, isCameraUse);
            intake.ShooterTest(power);
            telemetry.update();
        }
    }
    /*
    dist velocity pose
    1) 700 1650 0
    2) 1037 1750 0.5
    3) 1456 1790 0.5
    4) 1934 1900 0.6
    5) 2812 2050 0.95
     */
}
