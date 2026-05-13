package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class TeleOpBase extends LinearOpMode {
  Drivetrain drivetrain;
  
  @Override
  public void runOpMode() throws InterruptedException {
    drivetrain = new Drivetrain(this);
    waitForStart();
    drivetrain.update();
  }
  public abstract int getAllianceAprilTagID();
  
}
