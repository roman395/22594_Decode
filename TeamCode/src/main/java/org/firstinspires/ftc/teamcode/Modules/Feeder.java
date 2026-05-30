package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configs.FeederConfig;
import org.firstinspires.ftc.teamcode.StateMachine.IStateMachineCaller;
import org.firstinspires.ftc.teamcode.StateMachine.RobotStates;
import org.firstinspires.ftc.teamcode.Configs.HardwareNames;
import org.firstinspires.ftc.teamcode.Utils.MotorMaker;

public class Feeder extends Module implements IStateMachineCaller {
  private DcMotor feederMotor;
  private Rev2mDistanceSensor distanceSensor;
  private double currentSensorDistance;
  private Gamepad gamepad;
  
  public Feeder(LinearOpMode linearOpMode) {
    feederMotor = new MotorMaker(HardwareNames.FeederMotor, linearOpMode)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .setDirection(DcMotorSimple.Direction.REVERSE)
        .build();
    distanceSensor = linearOpMode.hardwareMap.get(Rev2mDistanceSensor.class, HardwareNames.RevDistanceSensor);
    gamepad = linearOpMode.gamepad1;
  }
  
  @Override
  public void update() {
    currentSensorDistance = distanceSensor.getDistance(DistanceUnit.MM);
  }
  
  @Override
  public void addTelemetry(Telemetry telemetry) {
    telemetry.addData("Feeder distance sensor:", currentSensorDistance);
  }
  
  @Override
  public void onCall(RobotStates state) {
    if ((state == RobotStates.SHOOTING || state == RobotStates.INTAKING) && gamepad.right_bumper)
      feederMotor.setPower(gamepad.right_bumper ? FeederConfig.MAX_FEEDER_SPEED : 0);
    else if(gamepad.left_bumper)
      feederMotor.setPower(gamepad.left_bumper ? -FeederConfig.MAX_FEEDER_SPEED : 0);
    else
      feederMotor.setPower(0);
  }
  
  @Override
  public RobotStates requestState(RobotStates currentState) {
    if (currentSensorDistance < FeederConfig.distanceWhenEmpty - FeederConfig.errorThreshold && currentState == RobotStates.INTAKING)
      return RobotStates.BALL_IN_FEEDER;
    return null;
  }
  
}
