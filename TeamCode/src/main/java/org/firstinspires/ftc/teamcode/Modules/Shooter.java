package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.Camera;
import org.firstinspires.ftc.teamcode.Configs.HardwareNames;
import org.firstinspires.ftc.teamcode.Configs.ShooterConfig;
import org.firstinspires.ftc.teamcode.StateMachine.IStateMachineCaller;
import org.firstinspires.ftc.teamcode.StateMachine.RobotStates;
import org.firstinspires.ftc.teamcode.Utils.MotorMaker;
import org.firstinspires.ftc.teamcode.Utils.PID;

public class Shooter extends Module implements IStateMachineCaller {
  private final DcMotorEx leftMotor, rightMotor;
  private double currentVelocity, targetVelocity;
  private final PID pidRegulator = new PID(ShooterConfig.COEFFICIENTS);
  private final Camera camera;
  
  public Shooter(LinearOpMode linearOpMode, Camera camera) {
    leftMotor = new MotorMaker(HardwareNames.ShootLeft, linearOpMode)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .buildEx();
    //Set as master for pid calculation
    rightMotor = new MotorMaker(HardwareNames.ShootRight, linearOpMode)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .buildEx();
    this.camera = camera;
  }
  
  @Override
  public void update() {
    currentVelocity = rightMotor.getVelocity();
    if (camera.getDistance() != -404)
      targetVelocity = camera.getDistance();
  }
  
  @Override
  public void addTelemetry(Telemetry telemetry) {
    telemetry.addData("Current velocity: ", currentVelocity);
    telemetry.addData("Target velocity: ", targetVelocity);
  }
  
  @Override
  public void onCall(RobotStates state) {
    if (state == RobotStates.SPOOLING || state == RobotStates.SHOOTING) {
      double power = pidRegulator.calculateVelocity(currentVelocity - targetVelocity, targetVelocity);
      leftMotor.setPower(power);
      rightMotor.setPower(power);
    }
  }
  
  @Override
  public RobotStates requestState(RobotStates currentState) {
    if (currentState == RobotStates.SPOOLING && Math.abs(currentVelocity - targetVelocity) < ShooterConfig.INACCURACY)
      return RobotStates.READY_TO_SHOOT;
    return null;
  }
  
}
