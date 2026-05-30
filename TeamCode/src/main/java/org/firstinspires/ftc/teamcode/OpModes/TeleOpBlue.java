package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "OpModes", name = "BLUE_TeleOp")
public class TeleOpBlue extends TeleOpBase {
  @Override
  public int getAllianceAprilTagID() {
    return 20;
  }
  @Override
  public Pose getGoalPose() {
    return null;
  }
  
}
