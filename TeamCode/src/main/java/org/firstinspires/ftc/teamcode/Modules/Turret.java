package org.firstinspires.ftc.teamcode.Modules;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Camera.Camera;
import org.firstinspires.ftc.teamcode.Configs.HardwareNames;
import org.firstinspires.ftc.teamcode.Configs.TurretConfig;
import org.firstinspires.ftc.teamcode.StateMachine.IStateMachineCaller;
import org.firstinspires.ftc.teamcode.StateMachine.RobotStates;
import org.firstinspires.ftc.teamcode.Utils.GlobalStorage;
import org.firstinspires.ftc.teamcode.Utils.PID;

public class Turret extends Module implements IStateMachineCaller {
  private final CRServo servo1, servo2;
  private final AnalogInput servoEncoder;
  private final Camera camera;
  private double headingError = 0;
  private double currentVoltage = 0;
  private int countOfFullTurns = 0;
  private Pose goalPose;
  private double centerPose = 2.263;
  private final PID pidRegulator = new PID(TurretConfig.COEFFICIENTS);
  private boolean isBlocked = false;
  
  public Turret(LinearOpMode linearOpMode, Camera camera, Pose goalPose) {
    servo1 = linearOpMode.hardwareMap.get(CRServo.class, HardwareNames.TurretServo1);
    servo2 = linearOpMode.hardwareMap.get(CRServo.class, HardwareNames.TurretServo2);
    servoEncoder = linearOpMode.hardwareMap.get(AnalogInput.class, HardwareNames.TurretServoEncoder1);
    this.camera = camera;
    this.goalPose = goalPose;
    
    servo1.setDirection(DcMotorSimple.Direction.REVERSE);
    servo2.setDirection(DcMotorSimple.Direction.REVERSE);
  }
  
  @Override
  public void update() {
    headingError = camera.getXHeading();
    currentVoltage = servoEncoder.getVoltage();
  }
  
  @Override
  public void addTelemetry(Telemetry telemetry) {
  
  }
  
  public void saveData() {
    GlobalStorage.lastTurretCenterPose = centerPose;
    GlobalStorage.lastTurretFullTurns = countOfFullTurns;
  }
  
  public void loadData() {
    centerPose = GlobalStorage.lastTurretCenterPose;
    countOfFullTurns = GlobalStorage.lastTurretFullTurns;
  }

  @Override
  public void onCall(RobotStates state) {

  }

  @Override
  public RobotStates requestState(RobotStates currentState) {
    return null;
  }
}
