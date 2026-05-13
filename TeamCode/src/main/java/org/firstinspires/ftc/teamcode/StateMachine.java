package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

public class StateMachine {
  List<IStateMachineCaller> listeners = new ArrayList<>();
  List<RobotStates> requestStates = new ArrayList<>();
  
  RobotStates currentState = RobotStates.IDLE;
  Gamepad gamepad;
  public StateMachine(LinearOpMode linearOpMode){
    gamepad = linearOpMode.gamepad1;
  }
  public void addListener(IStateMachineCaller listener) {
    listeners.add(listener);
  }
  
  public void update() {
    for (IStateMachineCaller listener : listeners) {
      requestStates.add(listener.requestState());
      listener.onCall(currentState);
    }
    switch(currentState){
      case IDLE:
        if(gamepad.circleWasPressed())
          currentState = RobotStates.SPOOLING;
        break;
      case SPOOLING:
        if(requestStates.contains(RobotStates.SHOOTING))
          currentState = RobotStates.SHOOTING;
        break;
      case SHOOTING:
        break;
    }
  }
  
}
