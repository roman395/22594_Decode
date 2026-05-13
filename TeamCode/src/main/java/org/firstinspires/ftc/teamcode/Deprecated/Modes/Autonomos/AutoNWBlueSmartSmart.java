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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous OpMode for the Big Red configuration, using Pedro Pathing.
 * This has been refactored to a sequential LinearOpMode structure for clarity and reliability,
 * and now properly integrates the Shooter and Intake modules.
 */
@Autonomous(name = "NW blue smart smart", group = "Autonomous")
public class AutoNWBlueSmartSmart extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;
    private Paths myPath;
    private ElapsedTime timer = new ElapsedTime();

    // The AprilTag ID for the red alliance backdrop
    private static final int APRILTAG_TARGET_ID = 20;
    private static final Pose goal = new Pose(15, 135);

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
        follower.setStartingPose(new Pose(25.9, 127.6, Math.toRadians(135)));

        panelsTelemetry.debug("Status", "Initialized and Ready");
        panelsTelemetry.update(telemetry);
        waitForStart();
        turret.resetStartPose();

        if (isStopRequested()) return;
        waitUntilTime(1500);
        follower.followPath(myPath.shootpreload);
        turret.setTargetPose(-10);
        waitUntilPathDone();
        waitUntilShootingDone();

        follower.followPath(myPath.takespike1);
        waitUntilPathDoneTaking();

        follower.followPath(myPath.shootspike1);
        shooter.startSpooling(1500);
        turret.setTargetPose(60);
        waitUntilPathDoneTaking();
        waitUntilShootingDone();

        follower.followPath(myPath.takespike2);
        waitUntilPathDoneTaking();

        follower.followPath(myPath.shootspike2);
        shooter.startSpooling(1700);
        turret.setTargetPose(40);
        waitUntilPathDoneTaking();
        waitUntilShootingDone();

        follower.followPath(myPath.takesmart);
        waitUntilPathDoneTaking();
        follower.followPath(myPath.smallLeave);
        waitUntilPathDoneTaking();
        waitUntilTime(1000 );

        follower.followPath(myPath.shootsmart);
        shooter.startSpooling(1700);
        turret.setTargetPose(-3);
        waitUntilPathDoneTaking();
        waitUntilShootingDone();
    }

    private void waitUntilPathDoneTaking() {
        while (opModeIsActive() && follower.isBusy()) {
            shooter.updateSpooling();
            follower.update();
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
        public PathChain leave;

        public Paths(Follower follower) {
            shootpreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.981, 127.655),

                                    new Pose(41.431, 111.036)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))

                    .build();

            takespike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(41.431, 111.036),
                                    new Pose(64.458, 77.450),
                                    new Pose(12.000, 80.642)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            shootspike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 80.642),

                                    new Pose(40.339, 92.569)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            takespike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(40.339, 92.569),
                                    new Pose(64.872, 56.358),
                                    new Pose(8.000, 60.904)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shootspike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.000, 60.904),
                                    new Pose(32.000, 67.322),
                                    new Pose(43.477, 88.684)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            takesmart = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(43.477, 88.684),
                                    new Pose(44.005, 63.991),
                                    new Pose(6.567, 62.898)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            smallLeave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7.567, 62.898),

                                    new Pose(7.567, 58.898)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();

            shootsmart = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(7.567, 58.898),

                                    new Pose(52.559, 105.143)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(50.559, 83.143),

                                    new Pose(47.339, 73.932)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(135))

                    .build();
        }
    }

}
