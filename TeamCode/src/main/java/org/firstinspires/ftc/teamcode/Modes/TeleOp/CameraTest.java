package org.firstinspires.ftc.teamcode.Modes.TeleOp;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.AprilTagsDetection;

@TeleOp
public class CameraTest extends LinearOpMode {
    Telemetry tel = PanelsTelemetry.INSTANCE.getFtcTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagsDetection april = new AprilTagsDetection(this,tel);
        waitForStart();
        PanelsCameraStream.INSTANCE.startStream(april.getPortal(),120);

        while (opModeIsActive()) {
            april.TeleOp();
        }
        PanelsCameraStream.INSTANCE.stopStream();

    }
}
