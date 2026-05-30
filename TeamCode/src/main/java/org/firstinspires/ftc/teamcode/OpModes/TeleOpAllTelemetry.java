package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "Debug", name = "DEBUG_TeleOp")
public class TeleOpAllTelemetry extends TeleOpBase {
  @Override
  public int getAllianceAprilTagID() {
    return 23;
  }
  
  @Override
  public void addTelemetry(){
    machine.addTelemetry(telemetry);
    shooter.addTelemetry(telemetry);
    intake.addTelemetry(telemetry);
    feeder.addTelemetry(telemetry);
    turret.addTelemetry(telemetry);
    drivetrain.addTelemetry(telemetry);
  }
  
  @Override
  public Pose getGoalPose() {
    return new Pose(0,0,0);
  }
  
}
