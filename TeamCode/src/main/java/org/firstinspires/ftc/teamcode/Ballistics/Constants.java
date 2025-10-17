package org.firstinspires.ftc.teamcode.Ballistics;

import org.firstinspires.ftc.teamcode.Ballistics.MathDouble.NullFunc;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.NullFuncMatrix;

public class Constants
{
    public static final NullFunc NullFunctionDouble = new NullFunc();
    public static final NullFuncMatrix NullFunctionMatrix = new NullFuncMatrix();
    public static final double DragBigConstant = 69, // TODO: Calculate constant, SI OR IMPERIAL????
            g = 9.81, // TODO: Use SI or imperial?
            Goal = 67, // TODO: Input distance to which throw the ball away from the wall
            Height = 41, // TODO: Input wall height
            HeightError = 30, // TODO: Input this, this === any room for the ball to not hit the wall
            BallRadius = 42, // SI??? IMPERIAL???
            Step = 0.01, // Step for DiffEq, Newton and integration
            Precision = 0.001; // Precision for Newton's method
}
