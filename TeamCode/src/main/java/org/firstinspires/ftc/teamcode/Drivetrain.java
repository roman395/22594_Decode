package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain extends Module {
  Follower follower;
  Gamepad gamepad;
  IMU imu;
  float X_DEBUG_START_POSE = 72, Y_DEBUG_START_POSE = 72, ANGLE_DEBUG_START_POSE = 90;
  String IMU_HARDWARE_NAME = "imu";
  
  public Drivetrain(LinearOpMode linearOpMode) {
    follower = Constants.createFollower(linearOpMode.hardwareMap);
    follower.setStartingPose(new Pose(X_DEBUG_START_POSE, Y_DEBUG_START_POSE, Math.toRadians(ANGLE_DEBUG_START_POSE)));
    follower.update();
    follower.startTeleOpDrive();
    follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -(gamepad.right_trigger - gamepad.left_trigger) * 0.5, true);
    gamepad = linearOpMode.gamepad1;
    imu = linearOpMode.hardwareMap.get(IMU.class, IMU_HARDWARE_NAME);
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
        RevHubOrientationOnRobot.UsbFacingDirection.UP
    )));
  }
  
  @Override
  public void update() {
    follower.update();
    if (gamepad.optionsWasPressed())
      imu.resetYaw();
    
  }
  
  @Override
  public void addTelemetry(Telemetry telemetry) {
    telemetry.addData("X:", follower.getPose().getX());
    telemetry.addData("Y:", follower.getPose().getY());
    telemetry.addData("Angle:", Math.toDegrees(follower.getPose().getHeading()));
  }
  
  public void loadStartPose() {
    follower.setStartingPose(GlobalStorage.lastPose);
  }
  
  public Pose getPose() {
    return follower.getPose();
  }
  
}
