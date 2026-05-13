package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Module {
  public Module() {
  }
  
  public abstract void update();
  
  public abstract void addTelemetry(Telemetry telemetry);
  
}
