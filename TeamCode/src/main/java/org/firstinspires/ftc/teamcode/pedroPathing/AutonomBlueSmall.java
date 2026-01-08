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

@Autonomous
public class AutonomBlueSmall extends LinearOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths;// Paths defined in the Paths class
    private boolean isShooterComplete;
    private Shooter shooter;
    private Intake intake;
    public static double intakeWorkTime, intakeWaitTime, intakePower;

    @Override
    public void runOpMode() throws InterruptedException {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(this, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intake = new Intake(this);
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

        public PathChain ShootPreload;
        public double ShootPreloadWait;
        public PathChain TakeSpike1;
        public PathChain Shoot1;
        public double Shoot1Wait;
        public PathChain TakeSpike2;
        public PathChain Shoot2;
        public double Shoot2Wait;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.000, 110.000), new Pose(37.000, 120.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(140))
                    .build();

            ShootPreloadWait = 4000;

            TakeSpike1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(37.000, 120.000),
                                    new Pose(22.000, 123.000),
                                    new Pose(24.000, 90.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(270))
                    .build();

            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 90.000), new Pose(36.000, 97.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            Shoot1Wait = 4000;

            TakeSpike2 = follower
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

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 65.000), new Pose(45.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            Shoot2Wait = 4000;

            Leave = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(45.000, 88.000), new Pose(45.000, 80.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(120))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState){
            case 0://Goto shoot preload
                follower.followPath(paths.ShootPreload);
                pathState = 1;
                break;
            case 1://Shoot preload
                if(follower.isBusy()) {
                    shooter.ResetTimer();
                    intake.ResetTimer();
                }
                if(!follower.isBusy() && !isShooterComplete) {
                    isShooterComplete = shooter.Autonom(paths.ShootPreloadWait, 20);
                    intake.Autonom(intakePower, intakeWorkTime, intakeWaitTime, 3);
                }
                if(isShooterComplete)
                {
                    isShooterComplete = false;
                    pathState = 2;
                    follower.followPath(paths.TakeSpike1);
                }
                break;
            case 2://Goto spike 1
                intake.Autonom(1);
                if(!follower.isBusy()){
                    intake.Autonom(0);
                    pathState = 3;
                }
                break;
            case 3://Goto shoot spike 1
                follower.followPath(paths.Shoot1);
                pathState = 4;
                break;
            case 4://Shoot spike 1
                if(follower.isBusy()) {
                    shooter.ResetTimer();
                    intake.ResetTimer();
                }
                if(!follower.isBusy() && !isShooterComplete) {
                    isShooterComplete = shooter.Autonom(paths.Shoot2Wait, 20);
                    intake.Autonom(intakePower, intakeWorkTime, intakeWaitTime, 3);
                }
                if(isShooterComplete)
                {
                    isShooterComplete = false;
                    pathState = 5;
                    follower.followPath(paths.Leave);
                }
                break;
        }
        return pathState;
    }
}