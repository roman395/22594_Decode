package org.firstinspires.ftc.teamcode.Deprecated.Modes.Autonomos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Deprecated.Modules.GlobalStorage;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Intake;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Turret;
import org.firstinspires.ftc.teamcode.Deprecated.pedroPathing.Constants;

/**
 * Autonomous OpMode for the Big Red configuration, using Pedro Pathing.
 * This has been refactored to a sequential LinearOpMode structure for clarity and reliability,
 * and now properly integrates the Shooter and Intake modules.
 */
@Autonomous(name = "NW red smart smart", group = "Autonomous")
public class AutoNWBRedSmartSmart extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;
    private Paths myPath;
    private ElapsedTime timer = new ElapsedTime();

    // The AprilTag ID for the red alliance backdrop
    private static final int APRILTAG_TARGET_ID = 24;
    private static final Pose goal = new Pose(130,132);

    @Override
    public void runOpMode() {
        // --- Initialization ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(this, APRILTAG_TARGET_ID, goal);
        intake = new Intake(this);
        turret = new Turret(this, goal);
        follower = Constants.createFollower(hardwareMap);
        myPath = new Paths(follower);

        // Set the starting pose for the robot
        follower.setStartingPose(new Pose(118.1, 127.6, Math.toRadians(45)));

        panelsTelemetry.debug("Status", "Initialized and Ready");
        panelsTelemetry.update(telemetry);
        waitForStart();
        turret.resetStartPose();

        if (isStopRequested()) return;
        waitUntilTime(1500);

        follower.followPath(myPath.shootpreload);
        turret.setTargetPose(5);
        waitUntilPathDone();
        waitUntilShootingDone();

        follower.followPath(myPath.takespike1);
        waitUntilPathDoneTaking();

        follower.followPath(myPath.shootspike1);
        shooter.startSpooling(1600);
        turret.setTargetPose(280);
        waitUntilPathDoneTaking();
        waitUntilShootingDone();

        follower.followPath(myPath.takespike2);
        waitUntilPathDoneTaking();

        follower.followPath(myPath.shootspike2);
        shooter.startSpooling(1800);
        turret.setTargetPose(260);
        waitUntilPathDoneTaking();
        waitUntilShootingDone();

        follower.followPath(myPath.takesmart);
        waitUntilPathDoneTaking();
        follower.followPath(myPath.smallLeave);
        waitUntilPathDoneTaking();
        waitUntilTime(1000);

        follower.followPath(myPath.shootsmart);
        shooter.startSpooling(2000);
        turret.setTargetPose(5);
        waitUntilPathDoneTaking();
        waitUntilShootingDone();

    }

    private void waitUntilPathDoneTaking() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            shooter.updateSpooling();
            intake.grabbingAutonomous();
            turret.saveData();
            GlobalStorage.lastPose = follower.getPose();
        }
        intake.stopAll();
    }

    private void waitUntilPathDone() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            turret.updateTurret(shooter.getBearing());
            turret.saveData();
            GlobalStorage.lastPose = follower.getPose();
        }

    }



    private void waitUntilShootingDone() {
        shooter.resetAutonomousShootingSequence();
        while (!shooter.runAutonomousShootingSequence()) {
            turret.updateTurret(shooter.getBearing());
            shooter.advancedTelemetry(telemetry);
            //turret.advancedTelemetry(telemetry);
            telemetry.update();
            follower.update();
            turret.saveData();
            GlobalStorage.lastPose = follower.getPose();
        }
    }

    private void waitUntilTime(double millis) {
        timer.reset();
        while (!(timer.milliseconds() > millis)) {
            follower.update();
            intake.grabbingAutonomous();
        }
    }



    public static class Paths {
        public PathChain shootpreload;
        public PathChain takespike1;
        public PathChain shootspike1;
        public PathChain takespike2;
        public PathChain shootspike2;
        public PathChain takesmart;
        public PathChain smallLeave;
        public PathChain shootsmart;

        public Paths(Follower follower) {
            shootpreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(118.100, 127.600),

                                    new Pose(102.569, 111.036)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(60))

                    .build();

            takespike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(102.569, 111.036),
                                    new Pose(79.542, 77.450),
                                    new Pose(128.000, 80.642)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))

                    .build();

            shootspike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.000, 80.642),

                                    new Pose(103.661, 92.569)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))

                    .build();

            takespike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(103.661, 92.569),
                                    new Pose(79.128, 56.358),
                                    new Pose(130.000, 60.904)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))

                    .build();

            shootspike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.000, 60.904),
                                    new Pose(112.000, 67.322),
                                    new Pose(100.523, 88.684)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))

                    .build();

            takesmart = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(100.523, 88.684),
                                    new Pose(99.995, 63.991),
                                    new Pose(137.433, 62.898)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(45))

                    .build();

            smallLeave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.433, 62.898),

                                    new Pose(136.433, 58.898)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(45))

                    .build();

            shootsmart = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(136.433, 58.898),

                                    new Pose(85.596, 103.191)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(65))

                    .build();
        }
    }

}
