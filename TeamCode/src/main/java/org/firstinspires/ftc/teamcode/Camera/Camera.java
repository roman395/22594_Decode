package org.firstinspires.ftc.teamcode.Camera;

public abstract class Camera {
  int tagId;
  public Camera(int tagId){
    this.tagId = tagId;
  }
  public abstract double getDistance();
  public abstract double getXHeading();
  public abstract void update();
}
