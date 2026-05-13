package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;

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
