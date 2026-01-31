package org.firstinspires.ftc.teamcode.Modules;

import com.bylazar.configurables.annotations.Configurable;

import org.opencv.core.Mat;

@Configurable
public class ShooterCalc {
    double x, y, g = 9.81;
    public static double teta = 0;

    double AngleCalc(double range, double elevation, double teta) {
        return Math.toDegrees(Math.atan(2 * elevation / range - Math.tan(Math.toDegrees(teta))));
    }

    double VelocityCalc(double range, double elevation, double teta) {
        double alpha = Math.toRadians(AngleCalc(range, elevation, teta));
        return Math.sqrt((g * range * range) / (2 * Math.cos(alpha) * Math.cos(alpha) * (range * Math.tan(alpha) - elevation)));
    }
}
