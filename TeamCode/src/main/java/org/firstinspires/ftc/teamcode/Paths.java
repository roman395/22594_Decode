package org.firstinspires.ftc.teamcode;
import pedro.pathing.follower.Follower;
import pedro.pathing.path.PathChain;
import pedro.pathing.path.patterns.BezierCurve;
import pedro.pathing.path.patterns.BezierLine;
import pedro.pathing.structures.Pose;
public class Paths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public Paths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.000, 8.000),
                        new Pose(23.128, 13.336),
                        new Pose(23.972, 35.451)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
        Path2 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(23.972, 35.451),
                        new Pose(56.216, 8.272)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.216, 8.272),
                        new Pose(24.141, 32.413),
                        new Pose(23.972, 60.436)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))
                .build();
        Path4 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(23.972, 60.436),
                        new Pose(55.372, 88.966)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
        Path5 = follower
                .pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.372, 88.966),
                        new Pose(59.761, 117.834),
                        new Pose(21.946, 118.002),
                        new Pose(23.972, 84.070)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                .build();
        Path6 = follower
                .pathBuilder()
                .addPath(new BezierLine(
                        new Pose(23.972, 84.070),
                        new Pose(43.048, 100.277)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                .build();
    }
}
