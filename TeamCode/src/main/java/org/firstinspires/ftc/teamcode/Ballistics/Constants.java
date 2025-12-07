package org.firstinspires.ftc.teamcode.Ballistics;

import org.firstinspires.ftc.teamcode.Ballistics.MathDouble.NullFunc;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.NullFuncMatrix;

public class Constants
{
    public static final NullFunc NullFunctionDouble = new NullFunc();
    public static final NullFuncMatrix NullFunctionMatrix = new NullFuncMatrix();
    public static final double DragConstant = 69, // TODO: Calculate constant, dimensionless
            AirDensity = 1.225, // kg / m^3
            g = 9.81, // m / s^2
            Height = 0.9845, // m
            HeightError = 0.01, // TODO: Input this, this === any room for the ball to not hit the wall, m
            BallRadius = 0.0635, // m
            BallMass = 0.0748, // kg
            CrossSectionalArea = 0.1995, // m^2
            Step = 0.01, // Step for DiffEq, Newton and integration
            Precision = 0.001; // Precision for Newton's method
}
