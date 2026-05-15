package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
  private PIDFCoefficients coefficients;
  private double lastError = 0;
  private double lastTime = 0;
  private double iOutput = 0;
  private double maxI = 1, minI = -1;
  private final ElapsedTime pidTimeer = new ElapsedTime();
  
  public PID(PIDFCoefficients pidfCoefficients) {
    this.coefficients = pidfCoefficients;
    pidTimeer.reset();
  }
  
  public PID setP(double p) {
    coefficients.p = p;
    return this;
  }
  
  public PID setI(double i) {
    coefficients.i = i;
    return this;
  }
  
  public PID setD(double d) {
    coefficients.d = d;
    return this;
  }
  
  public PID setF(double f) {
    coefficients.f = f;
    return this;
  }
  
  public PID setMaxI(double maxI) {
    this.maxI = maxI;
    return this;
  }
  
  public PID setMinI(double minI) {
    this.minI = minI;
    return this;
  }
  
  public double calculateVelocity(double error, double target) {
    double currentTime = pidTimeer.milliseconds();
    double fOutput = target * coefficients.f;
    double pOutput = error * coefficients.p;
    iOutput += (error * (currentTime - lastTime)) * coefficients.i;
    double dOutput = (error - lastError) / (currentTime - lastTime) * coefficients.d;
    
    iOutput = Math.clamp(iOutput, minI, maxI);
    lastError = error;
    lastTime = currentTime;
    return pOutput + iOutput + dOutput + fOutput;
  }
  
}
