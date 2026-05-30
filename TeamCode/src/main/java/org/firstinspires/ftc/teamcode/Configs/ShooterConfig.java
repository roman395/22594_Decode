package org.firstinspires.ftc.teamcode.Configs;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConfig {
  public static PIDFCoefficients COEFFICIENTS = new PIDFCoefficients(0.003, 0, 0, 0.00036);
  public static double INACCURACY = 35;
  public static double BANG_BANG_THRESHOLD = 100;
  
}
