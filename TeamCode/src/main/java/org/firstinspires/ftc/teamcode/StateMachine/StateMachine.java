package org.firstinspires.ftc.teamcode.StateMachine;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

public class StateMachine {
  List<IStateMachineCaller> listeners = new ArrayList<>();
  List<RobotStates> requestStates = new ArrayList<>();
  
  RobotStates currentState = RobotStates.IDLE;
  Gamepad gamepad;
  
  public StateMachine(LinearOpMode linearOpMode) {
    gamepad = linearOpMode.gamepad1;
  }
  
  public void addListener(IStateMachineCaller listener) {
    listeners.add(listener);
  }
  
  public void update() {
    requestStates.clear();
    for (IStateMachineCaller listener : listeners) {
      if (listener.requestState(currentState) != null)
        requestStates.add(listener.requestState(currentState));
      listener.onCall(currentState);
    }
    if (gamepad.circleWasPressed())
      currentState = RobotStates.SPOOLING;
    
    switch (currentState) {
      case IDLE:
        if (gamepad.right_bumper || gamepad.left_bumper)
          currentState = RobotStates.INTAKING;
        break;
      case INTAKING:
        if (requestStates.contains(RobotStates.BALL_IN_FEEDER))
          currentState = RobotStates.INTAKING_WITH_BALL_IN;
        if (!gamepad.right_bumper && !gamepad.left_bumper)
          currentState = RobotStates.IDLE;
        break;
      case INTAKING_WITH_BALL_IN:
        if (!requestStates.contains(RobotStates.BALL_IN_FEEDER))
          currentState = RobotStates.INTAKING;
        if (!gamepad.right_bumper && !gamepad.left_bumper)
          currentState = RobotStates.IDLE;
      case SPOOLING:
        if (requestStates.contains(RobotStates.READY_TO_SHOOT))
          currentState = RobotStates.SHOOTING;
        if (gamepad.squareWasPressed())
          currentState = RobotStates.IDLE;
        break;
      case SHOOTING:
        if (!requestStates.contains(RobotStates.READY_TO_SHOOT))
          currentState = RobotStates.SPOOLING;
        if (gamepad.squareWasPressed())
          currentState = RobotStates.IDLE;
        break;
    }
  }
  
}
