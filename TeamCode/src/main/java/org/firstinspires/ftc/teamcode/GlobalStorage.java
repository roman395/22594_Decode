package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class GlobalStorage {
    // These will persist between Auto and TeleOp
    public static Pose lastPose = new Pose(10, 10, Math.toRadians(90));
    public static double lastTurretFullTurns = 0;
    public static double lastTurretCenterPose = 2.263; // Default
    public static double lastDriftOffset = 0; // Calibration offset
}