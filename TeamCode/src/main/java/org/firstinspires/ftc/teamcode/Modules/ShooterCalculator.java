package org.firstinspires.ftc.teamcode.Modules;

public class ShooterCalculator {
  public static double QUADRATIC_COF = -79.0213, LINEAR_COF = 353.2320, FREE_COMPONENT = 1430.7563;
  public static double BREAK_POINT = 0.94, CLOSE_POS = 0.3, FAR_POSE = 1;
  
  public static double distToVelocityApprox(double x) {
    return QUADRATIC_COF * x * x * x + LINEAR_COF * x * x - 189.8235 * x + FREE_COMPONENT;
  }
  
  public static double distToWallPos(double x) {
    return x > BREAK_POINT ? FAR_POSE : CLOSE_POS;
  }
  
}
