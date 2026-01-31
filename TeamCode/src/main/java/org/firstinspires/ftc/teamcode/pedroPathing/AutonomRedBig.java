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
@Autonomous(name = "Pedro Pathing Autonomous RED BIG", group = "Autonomous")
public class AutonomRedBig extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;

    // The AprilTag ID for the red alliance backdrop
    private static final int APRILTAG_TARGET_ID = 24;

    @Override
    public void runOpMode() {
        // --- Initialization ---
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Shooter shooter = new Shooter(this, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        Intake intake = new Intake(this);
        follower = Constants.createFollower(hardwareMap);
        Paths paths = new Paths(follower);

        // Set the starting pose for the robot
        follower.setStartingPose(new Pose(129, 110, Math.toRadians(-90)));

        panelsTelemetry.debug("Status", "Initialized and Ready");
        panelsTelemetry.update(telemetry);

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
        shooter.SetCoefficientsAuto(2000,0);
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


        // 4. Первый выстрел
        panelsTelemetry.debug("State", "Driving to Cycle 1 Shot Position");
        follower.followPath(paths.Shoot1);
        waitUntilPathDone();
        intake.stop(); // ВЫКЛЮЧАЕМ после движения
        shooter.stopMotors();
        panelsTelemetry.debug("State", "Shooting Cycle 1");
        shooter.resetAutonomousShootingSequence();
        shooter.SetCoefficientsAuto(2000,0.1);
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


        // 6. Второй выстрел
        panelsTelemetry.debug("State", "Driving to Cycle 2 Shot Position");
        follower.followPath(paths.Shoot2);
        waitUntilPathDone();
        intake.stop(); // ВЫКЛЮЧАЕМ после движения
        shooter.stopMotors();

        panelsTelemetry.debug("State", "Shooting Cycle 2");
        shooter.resetAutonomousShootingSequence();
        shooter.SetCoefficientsAuto(2000,0.2);
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


        // 8. Третий выстрел
        panelsTelemetry.debug("State", "Driving to Cycle 3 Shot Position");
        follower.followPath(paths.Shoot3);
        waitUntilPathDone();
        intake.stop(); // ВЫКЛЮЧАЕМ после движения
        shooter.stopMotors();

        panelsTelemetry.debug("State", "Shooting Cycle 3");
        shooter.resetAutonomousShootingSequence();
        shooter.SetCoefficientsAuto(0,0);
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
                                    new Pose(129.000, 110.000),

                                    new Pose(102.000, 117.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-140))

                    .build();

            TakeSpike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(107.000, 120.000),
                                    new Pose(76.966, 95.804),
                                    new Pose(138.000, 77.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-140), Math.toRadians(0))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 84.000),

                                    new Pose(106.000, 98.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-130))

                    .build();

            TakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(106.000, 98.000),
                                    new Pose(86.080, 61.794),
                                    new Pose(128.000, 52.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(0))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 59.500),

                                    new Pose(97.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-130))

                    .build();

            TakeSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(97.000, 90.000),
                                    new Pose(80.913, 40.656),
                                    new Pose(133.000, 20.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(0))

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 35.500),

                                    new Pose(83.801, 85.511)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))

                    .build();

            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(83.801, 85.511),

                                    new Pose(85.053, 67.448)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(230))

                    .build();
        }
    }

}
