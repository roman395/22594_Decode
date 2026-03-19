package org.firstinspires.ftc.teamcode.Modes.Autonomos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous OpMode for the Big Red configuration, using Pedro Pathing.
 * This has been refactored to a sequential LinearOpMode structure for clarity and reliability,
 * and now properly integrates the Shooter and Intake modules.
 */
@Autonomous(name = "NW blue", group = "Autonomous")
public class AutoNWBlue extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;
    private Paths myPath;

    // The AprilTag ID for the red alliance backdrop
    private static final int APRILTAG_TARGET_ID = 20;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(this, APRILTAG_TARGET_ID);
        intake = new Intake(this);
        turret = new Turret(this);
        follower = Constants.createFollower(hardwareMap);
        myPath = new Paths(follower);

        // Set the starting pose for the robot
        follower.setStartingPose(new Pose(34, 135.5, Math.toRadians(180)));

        panelsTelemetry.debug("Status", "Initialized and Ready");
        panelsTelemetry.update(telemetry);
        waitForStart();
        turret.resetStartPose();
        if (isStopRequested()) return;

        follower.followPath(myPath.shootpreload);
        shooter.startSpooling(1600);
        waitUntilPathDone();
        waitUntilShootingDone(5);

        follower.followPath(myPath.takespike1);
        waitUntilPathDoneTaking();

        follower.followPath(myPath.shootspike1);
        shooter.startSpooling(1600);
        waitUntilPathDone();
        waitUntilShootingDone(60);

        follower.followPath(myPath.takespike2);
        waitUntilPathDoneTaking();


        follower.followPath(myPath.shootspike2);
        shooter.startSpooling(1800);
        waitUntilShootingDone(40);

        follower.followPath(myPath.leave);
        waitUntilPathDoneFinal();

    }

    private void waitUntilPathDoneTaking() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            intake.grabbingAutonomous();
        }
        intake.stopAll();
    }

    private void waitUntilPathDone() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    private void waitUntilShootingDone(double angle) {
        shooter.resetAutonomousShootingSequence();
        while (!shooter.runAutonomousShootingSequence()) {
            //turret.autonomousController(shooter.getBearing(), angle);
            shooter.advancedTelemetry();
            turret.advancedTelemetry(telemetry);
            telemetry.update();
            follower.update();
        }
    }

    private void waitUntilPathDoneFinal() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            turret.goToStart();
        }
    }


    public static class Paths {
        public PathChain shootpreload;
        public PathChain takespike1;
        public PathChain shootspike1;
        public PathChain takespike2;
        public PathChain shootspike2;
        public PathChain leave;

        public Paths(Follower follower) {
            shootpreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(34.000, 135.500),

                                    new Pose(43.000, 114.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            takespike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(43.000, 114.000),
                                    new Pose(73.349, 76.753),
                                    new Pose(15.500, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            shootspike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.500, 85.000),

                                    new Pose(46.000, 88.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            takespike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(46.000, 88.000),
                                    new Pose(64.872, 56.358),
                                    new Pose(14, 64.904)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shootspike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.849, 64.904),
                                    new Pose(32.000, 67.322),
                                    new Pose(55.661, 89.730)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.661, 89.730),

                                    new Pose(33.937, 78.194)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }


}
