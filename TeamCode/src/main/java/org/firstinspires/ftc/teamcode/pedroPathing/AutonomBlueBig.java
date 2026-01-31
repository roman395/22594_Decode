package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
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
 * Autonomous OpMode for the Big Red configuration, using Pedro Pathing.
 * This has been refactored to a sequential LinearOpMode structure for clarity and reliability,
 * and now properly integrates the Shooter and Intake modules.
 */
@Autonomous(name = "Pedro Pathing Autonomous Blue BIG", group = "Autonomous")
public class AutonomBlueBig extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private Shooter shooter;
    private Intake intake;

    // The AprilTag ID for the red alliance backdrop
    private static final int APRILTAG_TARGET_ID = 20;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(this, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intake = new Intake(this);
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        // Set the starting pose for the robot
        follower.setStartingPose(new Pose(15, 110, Math.toRadians(270)));

        panelsTelemetry.debug("Status", "Initialized and Ready");
        panelsTelemetry.update(telemetry);
        shooter.useOldApprox(true);
        waitForStart();

        if (isStopRequested()) return;

        // --- Autonomous Sequence ---

        // 1. Предзагрузка
        panelsTelemetry.debug("State", "Driving to Preload Shot Position");
        follower.followPath(paths.ShootPreload);
        waitUntilPathDone();

        // 2. Выстрел предзагрузки
        panelsTelemetry.debug("State", "Shooting Preload");
        shooter.resetAutonomousShootingSequence();
        while (opModeIsActive() && !shooter.runAutonomousShootingSequence(intake, APRILTAG_TARGET_ID)) {
            follower.update();
            updateTelemetry();
        }

        // 3. Первый захват
        panelsTelemetry.debug("State", "Driving to Spike 1 & Intaking");
        intake.ShooterEnable(1); // ВКЛЮЧАЕМ перед движением
        shooter.reverse(-0.8); // Реверс шутера
        follower.followPath(paths.TakeSpike1);
        waitUntilPathDone();
        intake.ShooterEnable(0.6); // ВКЛЮЧАЕМ перед движением
        shooter.reverse(-0.7); // Реверс шутера
        // 4. Первый выстрел
        panelsTelemetry.debug("State", "Driving to Cycle 1 Shot Position");
        follower.followPath(paths.Shoot1);
        waitUntilPathDone();
        intake.stop(); // ВЫКЛЮЧАЕМ после движения
        shooter.stopMotors();
        panelsTelemetry.debug("State", "Shooting Cycle 1");
        shooter.resetAutonomousShootingSequence();
        while (opModeIsActive() && !shooter.runAutonomousShootingSequence(intake, APRILTAG_TARGET_ID)) {
            follower.update();
            updateTelemetry();
        }

        // 5. Второй захват
        panelsTelemetry.debug("State", "Driving to Spike 2 & Intaking");
        intake.ShooterEnable(1); // ВКЛЮЧАЕМ перед движением
        shooter.reverse(-0.8);
        follower.followPath(paths.TakeSpike2);
        waitUntilPathDone();
        intake.ShooterEnable(0.6); // ВКЛЮЧАЕМ перед движением
        shooter.reverse(-0.7); // Реверс шутера

        // 6. Второй выстрел
        panelsTelemetry.debug("State", "Driving to Cycle 2 Shot Position");
        follower.followPath(paths.Shoot2);
        waitUntilPathDone();
        intake.stop(); // ВЫКЛЮЧАЕМ после движения
        shooter.stopMotors();

        panelsTelemetry.debug("State", "Shooting Cycle 2");
        shooter.resetAutonomousShootingSequence();
        while (opModeIsActive() && !shooter.runAutonomousShootingSequence(intake, APRILTAG_TARGET_ID)) {
            follower.update();
            updateTelemetry();
        }

        // 7. Третий захват
        panelsTelemetry.debug("State", "Driving to Spike 3 & Intaking");
        intake.ShooterEnable(1); // ВКЛЮЧАЕМ перед движением
        shooter.reverse(-0.8);
        follower.followPath(paths.TakeSpike3);
        waitUntilPathDone();
        intake.ShooterEnable(0.6); // ВКЛЮЧАЕМ перед движением
        shooter.reverse(-0.7); // Реверс шутера

        // 8. Третий выстрел
        panelsTelemetry.debug("State", "Driving to Cycle 3 Shot Position");
        follower.followPath(paths.Shoot3);
        waitUntilPathDone();
        intake.stop(); // ВЫКЛЮЧАЕМ после движения
        shooter.stopMotors();

        panelsTelemetry.debug("State", "Shooting Cycle 3");
        shooter.resetAutonomousShootingSequence();
        while (opModeIsActive() && !shooter.runAutonomousShootingSequence(intake, APRILTAG_TARGET_ID)) {
            follower.update();
            updateTelemetry();
        }

        // 9. Парковка
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
        //panelsTelemetry.debug("X", follower.getPose().getX());
        //panelsTelemetry.debug("Y", follower.getPose().getY());
        //panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        //panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);
    }

    // Inner class for defining paths remains the same



    public static class Paths {
        public PathChain ShootPreload;
        public PathChain TakeSpike1;
        public PathChain Shoot1;
        public PathChain TakeSpike2;
        public PathChain Shoot2;
        public PathChain TakeSpike3;
        public PathChain Shoot3;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 110.000),

                                    new Pose(45.000, 110.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-40))

                    .build();

            TakeSpike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(37.000, 120.000),
                                    new Pose(72.034, 95.804),
                                    new Pose(15.000, 82.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(180))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 77.000),

                                    new Pose(38.000, 98.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))

                    .build();

            TakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(38.000, 98.000),
                                    new Pose(62.920, 61.794),
                                    new Pose(15.000, 56.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(180))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 52.000),

                                    new Pose(47.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(310))

                    .build();

            TakeSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(47.000, 90.000),
                                    new Pose(68.087, 43.656),
                                    new Pose(15.000, 30.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(180))

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 28.000),

                                    new Pose(60.199, 85.511)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-40))

                    .build();

            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.199, 85.511),

                                    new Pose(58.947, 67.448)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-40))

                    .build();
        }
    }



}
