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

import org.firstinspires.ftc.teamcode.GlobalStorage;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Intake;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Deprecated.Modules.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous OpMode for the Big Red configuration, using Pedro Pathing.
 * This has been refactored to a sequential LinearOpMode structure for clarity and reliability,
 * and now properly integrates the Shooter and Intake modules.
 */
@Autonomous(name = "red faaa", group = "Autonomous")
public class AutoFarRed extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;
    private Paths myPath;

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
        follower.setStartingPose(new Pose(85, 8, Math.toRadians(90)));

        panelsTelemetry.debug("Status", "Initialized and Ready");
        panelsTelemetry.update(telemetry);
        waitForStart();
        turret.resetStartPose();
        if (isStopRequested()) return;

        follower.followPath(myPath.Path1);
        waitUntilPathDone();
        shooter.resetAutonomousShootingSequence();
        while (!shooter.runAutonomousShootingSequence()) {
            //turret.autonomousController(shooter.getBearing(), 5);
            shooter.advancedTelemetry(telemetry);
            //turret.advancedTelemetry(telemetry);
            telemetry.update();
            follower.update();
            turret.saveData();
            GlobalStorage.lastPose = follower.getPose();
        }
        follower.followPath(myPath.Path2);
        intake.grabbingAutonomous();
        waitUntilPathDone();


    }

    private void waitUntilPathDone() {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            turret.saveData();
            GlobalStorage.lastPose = follower.getPose();
        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.211, 8.349),

                                    new Pose(85.559, 20.136)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(85.559, 20.136),
                                    new Pose(90.230, 8.897),
                                    new Pose(134.896, 9.010)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-5))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.896, 9.010),
                                    new Pose(90.161, 9.023),
                                    new Pose(108.985, 40.232)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-5), Math.toRadians(70))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(108.985, 40.232),
                                    new Pose(90.103, 8.586),
                                    new Pose(134.872, 9.109)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-5))

                    .build();
        }
    }


}
