package org.firstinspires.ftc.teamcode.Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Camera.AprilTagsDetection;

@TeleOp
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagsDetection april = new AprilTagsDetection(this, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            april.TeleOp();
            FtcDashboard.getInstance().startCameraStream(april.getPortal(), 144);
        }
    }
}
