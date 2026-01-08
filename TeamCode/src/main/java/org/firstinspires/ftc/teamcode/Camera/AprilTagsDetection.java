package org.firstinspires.ftc.teamcode.Camera;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

public class AprilTagsDetection {
    private AprilTagProcessor aprilTag;
    private HardwareMap hmap;
    private VisionPortal visionPortal;
    Telemetry dash;

    public AprilTagsDetection(LinearOpMode mode, Telemetry dash) {
        this.dash = dash;
        hmap = mode.hardwareMap;
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()
                .setOutputUnits(DistanceUnit.MM, AngleUnit.DEGREES)
                .setCameraPose(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0))

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hmap.get(WebcamName.class, RobotConstants.Camera));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(false);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add dash about AprilTag detections.
     */
    public void dashAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        dash.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                dash.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                dash.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (MM)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                dash.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                dash.addLine(String.format("RBE %6.1f %6.1f %6.1f  (MM, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                dash.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                dash.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to dash
        dash.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        dash.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        dash.addLine("RBE = Range, Bearing & Elevation");
        dash.update();

    }   // end method dashAprilTag()

    private List<AprilTagDetection> currentDetections;
    private AprilTagDetection lastDetection;

    public void Update() {
        currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections)
            if (detection.metadata != null)
                lastDetection = detection;
    }

    public void TeleOp() {
        dashAprilTag();
    }

    public double GetDistance(int tagID) {
        if (lastDetection != null && lastDetection.id == tagID)
            return lastDetection.ftcPose.range;
        else
            return -1;
    }

    public VisionPortal getPortal() {
        return visionPortal;
    }

    public double GetBearing(int id) {
        currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections)
            if (detection.metadata != null && detection.id==id)
                return detection.ftcPose.bearing;
        return -999;
    }

    public double GetElevation() {
        return lastDetection.ftcPose.elevation;
    }
}
