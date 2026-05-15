package org.firstinspires.ftc.teamcode.Configs;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class TurretConfig {
  public static double MAX_ANGLE = 335;
  public static double MIN_ANGLE = -20;
  public static double SERVO_RANGE = 370;
  public static double MAX_VOLTAGE = 3.3;
  public static double VOLTAGE_TO_ANGLE = SERVO_RANGE / MAX_VOLTAGE;
  public static PIDFCoefficients COEFFICIENTS = new PIDFCoefficients(0.005, 0, 0.0005, 0.08);
}
