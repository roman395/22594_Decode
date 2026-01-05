package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class AutonomRedBig extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15, 110, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);
        }
    }

    public static class Paths {

        public PathChain Takespike1;
        public PathChain Shoot1;
        public PathChain Takespike2;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Takespike1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.000, 110.000),
                                    new Pose(25.000, 101.000),
                                    new Pose(24.000, 90.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                    .build();

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 90.000), new Pose(36.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            Takespike2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(36.000, 97.000),
                                    new Pose(42.000, 93.000),
                                    new Pose(24.000, 65.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(270))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 65.000), new Pose(45.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 88.000), new Pose(45.000, 80.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(120))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}