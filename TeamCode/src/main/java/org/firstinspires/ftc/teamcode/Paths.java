package org.firstinspires.ftc.teamcode;
import pedro.pathing.follower.Follower;
import pedro.pathing.path.PathChain;
import pedro.pathing.path.patterns.BezierCurve;
import pedro.pathing.path.patterns.BezierLine;
import pedro.pathing.structures.Pose;
public static class Paths {

  public PathChain Path1;
  public PathChain Path2;
  public PathChain Path3;
  public PathChain Path5;
  public PathChain Path6;
  public PathChain Path7;
  public PathChain Path7;
  public PathChain Path8;

  public Paths(Follower follower) {
    Path1 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          new Pose(56.000, 8.000),
          new Pose(23.128, 13.336),
          new Pose(23.972, 35.451)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
      .build();

    Path2 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(23.972, 35.451), new Pose(49.632, 12.830))
      )
      .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
      .build();

    Path3 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          new Pose(49.632, 12.830),
          new Pose(24.141, 32.413),
          new Pose(23.972, 60.436)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))
      .build();

    Path5 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          new Pose(23.972, 60.436),
          new Pose(40.347, 65.838),
          new Pose(45.411, 88.122)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
      .build();

    Path6 = follower
      .pathBuilder()
      .addPath(
        new BezierCurve(
          new Pose(45.411, 88.122),
          new Pose(44.905, 113.275),
          new Pose(22.115, 106.185),
          new Pose(23.972, 94.199)
        )
      )
      .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
      .build();

    Path7 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(23.972, 94.199), new Pose(23.803, 84.408))
      )
      .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
      .build();

    Path7 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(23.803, 84.408), new Pose(36.464, 97.576))
      )
      .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
      .build();

    Path8 = follower
      .pathBuilder()
      .addPath(
        new BezierLine(new Pose(36.464, 97.576), new Pose(34.438, 95.550))
      )
      .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
      .build();
  }
}
