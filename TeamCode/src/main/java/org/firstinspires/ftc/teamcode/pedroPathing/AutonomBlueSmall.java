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


public class AutonomBlueSmall extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

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
        public PathChain Shoot2;
        public PathChain Leave;

        public Paths(Follower follower) {
            Takespike1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.000),
                                    new Pose(21.000, 9.000),
                                    new Pose(24.000, 32.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 32.000), new Pose(56.000, 16.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            Takespike2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(56.000, 16.000),
                                    new Pose(30.000, 23.000),
                                    new Pose(24.000, 56.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(90))
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 56.000), new Pose(59.000, 76.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(124))
                    .build();

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 76.000), new Pose(59.000, 64.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(124))
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