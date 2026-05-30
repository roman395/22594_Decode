package org.firstinspires.ftc.teamcode.OpModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.LimeLightCamera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Feeder;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Shooter;
import org.firstinspires.ftc.teamcode.Modules.Turret;
import org.firstinspires.ftc.teamcode.StateMachine.StateMachine;

public abstract class TeleOpBase extends LinearOpMode {
  Drivetrain drivetrain;
  StateMachine machine;
  Intake intake;
  Feeder feeder;
  Turret turret;
  Shooter shooter;
  LimeLightCamera limeLightCamera;
  
  @Override
  public void runOpMode() throws InterruptedException {
    
    drivetrain = new Drivetrain(this);
    machine = new StateMachine(this);
    intake = new Intake(this);
    feeder = new Feeder(this);
    limeLightCamera = new LimeLightCamera(getAllianceAprilTagID(), this);
    //turret = new Turret(this, limeLightCamera, getGoalPose());
    shooter = new Shooter(this, limeLightCamera);
    
    machine.addListener(intake);
    machine.addListener(feeder);
    machine.addListener(turret);
    machine.addListener(shooter);
    
    waitForStart();
    
    while (opModeIsActive()) {
      drivetrain.update();
      machine.update();
      intake.update();
      feeder.update();
      limeLightCamera.update();
      turret.update();
      shooter.update();
      addTelemetry();
      telemetry.update();
    }
  }
  
  public abstract int getAllianceAprilTagID();
  
  public abstract Pose getGoalPose();
  public void addTelemetry(){
    machine.addTelemetry(telemetry);
  }
  
}
