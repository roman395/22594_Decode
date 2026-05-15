package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorMaker {
  private final String motorName;
  private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
  private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
  private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
  private final LinearOpMode linearOpMode;
  
  public MotorMaker(String motorName, LinearOpMode linearOpMode) {
    this.motorName = motorName;
    this.linearOpMode = linearOpMode;
  }
  
  public MotorMaker setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
    this.zeroPowerBehavior = zeroPowerBehavior;
    return this;
  }
  
  public MotorMaker setRunMode(DcMotor.RunMode runMode) {
    this.runMode = runMode;
    return this;
  }
  
  public MotorMaker setDirection(DcMotorSimple.Direction direction) {
    this.direction = direction;
    return this;
  }
  
  public DcMotor build() {
    DcMotor motor = linearOpMode.hardwareMap.get(DcMotor.class, motorName);
    motor.setMode(runMode);
    motor.setZeroPowerBehavior(zeroPowerBehavior);
    motor.setDirection(direction);
    return motor;
  }
  
  public DcMotorEx buildEx() {
    DcMotorEx motorEx = linearOpMode.hardwareMap.get(DcMotorEx.class, motorName);
    motorEx.setMode(runMode);
    motorEx.setZeroPowerBehavior(zeroPowerBehavior);
    motorEx.setDirection(direction);
    return motorEx;
  }
  
}
