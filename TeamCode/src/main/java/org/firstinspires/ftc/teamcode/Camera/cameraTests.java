package org.firstinspires.ftc.teamcode.Camera;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Mecanum;

@TeleOp
public class cameraTests extends LinearOpMode {
    Mecanum drive;
    AprilTagsDetection detect;
    @Override
    public void runOpMode() throws InterruptedException {
        detect = new AprilTagsDetection(this, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        drive = new Mecanum(this);
        PanelsCameraStream.INSTANCE.startStream(detect.getPortal(), 30);
        waitForStart();
        while (opModeIsActive()){
            detect.dashAprilTag();
            drive.TeleOp();
        }
    }
}
