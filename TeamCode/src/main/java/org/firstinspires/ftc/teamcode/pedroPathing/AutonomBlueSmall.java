package org.firstinspires.ftc.teamcode.pedroPathing;

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

/**
 * Autonomous OpMode for the Small Blue configuration, using Pedro Pathing.
 * This has been refactored to a sequential LinearOpMode structure for clarity and reliability.
 */
@Autonomous
public class AutonomBlueSmall extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private Shooter shooter;
    private Intake intake;

    // The AprilTag ID for the blue alliance backdrop
    private static final int APRILTAG_TARGET_ID = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(this, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intake = new Intake(this);
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower); // Build paths

        // The starting pose must match the start of the first path.
        follower.setStartingPose(new Pose(15,110,Math.toRadians(270)));
        panelsTelemetry.debug("Status", "Initialized and Ready");
        panelsTelemetry.update(telemetry);

        waitForStart();

        if (isStopRequested()) return;

        // --- Autonomous Sequence ---

        // 1. Drive to preload shooting position
        panelsTelemetry.debug("State", "Driving to Preload Shot Position");
        follower.followPath(paths.ShootPreload);
        waitUntilPathDone();

        // 2. Shoot the preloaded pixel
        panelsTelemetry.debug("State", "Shooting Preload");
        shooter.resetAutonomousShootingSequence();
        while (opModeIsActive() && !shooter.runAutonomousShootingSequence(intake, APRILTAG_TARGET_ID)) {
            follower.update(); // Keep odometry updated during actions
            updateTelemetry();
        }

        // 3. Drive to the first spike mark and run intake to grab a pixel
        panelsTelemetry.debug("State", "Driving to Spike 1 & Intaking");
        follower.followPath(paths.TakeSpike1);
        intake.run(); // Start intake while driving
        waitUntilPathDone();
        intake.stop(); // Stop intake after reaching the spike

        // 4. Drive to the next shooting position
        panelsTelemetry.debug("State", "Driving to Cycle 1 Shot Position");
        follower.followPath(paths.Shoot1);
        waitUntilPathDone();

        // 5. Shoot the pixel from the first cycle
        panelsTelemetry.debug("State", "Shooting Cycle 1");
        shooter.resetAutonomousShootingSequence();
        while (opModeIsActive() && !shooter.runAutonomousShootingSequence(intake, APRILTAG_TARGET_ID)) {
            follower.update();
            updateTelemetry();
        }
        
        // The original OpMode ended here, but had paths for a second cycle and leaving.
        // The following steps are based on the defined, but unused, paths.

        // 6. Drive to the second spike mark and run intake
        panelsTelemetry.debug("State", "Driving to Spike 2 & Intaking");
        follower.followPath(paths.TakeSpike2);
        intake.run();
        waitUntilPathDone();
        intake.stop();

        // 7. Drive to the final shooting position
        panelsTelemetry.debug("State", "Driving to Cycle 2 Shot Position");
        follower.followPath(paths.Shoot2);
        waitUntilPathDone();

        // 8. Shoot the final pixel
        panelsTelemetry.debug("State", "Shooting Cycle 2");
        shooter.resetAutonomousShootingSequence();
        while (opModeIsActive() && !shooter.runAutonomousShootingSequence(intake, APRILTAG_TARGET_ID)) {
            follower.update();
            updateTelemetry();
        }

        // 9. Park
        panelsTelemetry.debug("State", "Parking");
        follower.followPath(paths.Leave);
        waitUntilPathDone();

        panelsTelemetry.debug("Status", "Autonomous Complete!");
        panelsTelemetry.update(telemetry);
    }

    /**
     * Waits until the follower is no longer busy, updating it in a loop.
     */
    private void waitUntilPathDone() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
        }
    }

    /**
     * Updates telemetry with follower status for debugging.
     */
    private void updateTelemetry() {
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    // Inner class for defining paths remains the same
    public static class Paths {

        public PathChain ShootPreload;
        public PathChain TakeSpike1;
        public PathChain Shoot1;
        public PathChain TakeSpike2;
        public PathChain Shoot2;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 110.000, Math.toRadians(270)), new Pose(37.000, 120.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(140))
                    .build();

            TakeSpike1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(37.000, 120.000, Math.toRadians(140)),
                                    new Pose(22.000, 123.000),
                                    new Pose(24.000, 90.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(270))
                    .build();

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 90.000, Math.toRadians(270)), new Pose(36.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            TakeSpike2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(36.000, 97.000, Math.toRadians(120)),
                                    new Pose(42.000, 93.000),
                                    new Pose(24.000, 65.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(270))
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 65.000, Math.toRadians(270)), new Pose(45.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 88.000, Math.toRadians(120)), new Pose(45.000, 80.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(120))
                    .build();
        }
    }
}
