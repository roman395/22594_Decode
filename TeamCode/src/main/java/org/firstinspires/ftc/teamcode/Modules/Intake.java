package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configs.HardwareNames;
import org.firstinspires.ftc.teamcode.Configs.IntakeConfig;
import org.firstinspires.ftc.teamcode.StateMachine.IStateMachineCaller;
import org.firstinspires.ftc.teamcode.StateMachine.RobotStates;
import org.firstinspires.ftc.teamcode.Utils.MotorMaker;

public class Intake extends Module implements IStateMachineCaller {
  private final DcMotor intakeMotor;
  private final Gamepad gamepad;
  
  public Intake(LinearOpMode linearOpMode) {
    intakeMotor = new MotorMaker(HardwareNames.IntakeMotor, linearOpMode)
        .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        .build();
    gamepad = linearOpMode.gamepad1;
  }
  
  @Override
  public void update() {
  
  }
  
  @Override
  public void addTelemetry(Telemetry telemetry) {
  
  }
  
  @Override
  public void onCall(RobotStates state) {
    if (state != RobotStates.SPOOLING)
      intakeMotor.setPower(gamepad.right_bumper ? IntakeConfig.MAX_INTAKE_SPEED : 0);
    else
      intakeMotor.setPower(0);
  }
  
  @Override
  public RobotStates requestState(RobotStates currentState) {
    return null;
  }
  
}
