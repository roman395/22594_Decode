package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.IStateMachineCaller;
import org.firstinspires.ftc.teamcode.StateMachine.RobotStates;
import org.firstinspires.ftc.teamcode.Utils.HardwareNames;
import org.firstinspires.ftc.teamcode.Utils.MotorMaker;

public class Feeder extends Module implements IStateMachineCaller {
  private DcMotor feederMotor;
  private Rev2mDistanceSensor distanceSensor;
  private double currentSensorDistance;
  
  public Feeder(LinearOpMode linearOpMode) {
    feederMotor = new MotorMaker(HardwareNames.FeederMotor, linearOpMode)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .build();
    distanceSensor = linearOpMode.hardwareMap.get(Rev2mDistanceSensor.class, HardwareNames.RevDistanceSensor);
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
  
  }
  @Override
  public RobotStates requestState() {
    return null;
  }
  
}
