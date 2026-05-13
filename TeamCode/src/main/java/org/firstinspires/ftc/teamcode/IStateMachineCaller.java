package org.firstinspires.ftc.teamcode;

public interface IStateMachineCaller {
   void onCall(RobotStates state);
   RobotStates requestState();
}
