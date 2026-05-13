package org.firstinspires.ftc.teamcode.StateMachine;

public interface IStateMachineCaller {
   void onCall(RobotStates state);
   RobotStates requestState();
}
