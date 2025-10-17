package org.firstinspires.ftc.teamcode.Ballistics.MathMatrix;

import java.util.OptionalDouble;

public abstract class MatrixFunction
{
    public abstract Matrix execute(MatrixFunction func, Matrix... args);
    public double executeIndexed(int x, int y, MatrixFunction func, Matrix... args)
    {
        Matrix retval;
        retval = execute(func, args);
        return retval.element(x, y, OptionalDouble.empty());
    }
}
