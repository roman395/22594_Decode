package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "OpModes", name = "RED_TeleOp")
public class TeleOpRed extends TeleOpBase {
  @Override
  public int getAllianceAprilTagID() {
    return 24;
  }
  
  @Override
  public Pose getGoalPose() {
    return null;
  }
  
}
