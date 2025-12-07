package org.firstinspires.ftc.teamcode.Ballistics.MathDouble;

import static org.firstinspires.ftc.teamcode.Ballistics.Constants.NullFunctionDouble;

public class Integral extends Function
{
    @Override
    public double execute(Function func, double... args) // args = { lower_bound, upper_bound, step }
    {
        double area = 0;

        for (double i = args[0]; i < Math.abs(args[1] - args[0] - args[2]); i += args[2]) // TRAPEZOID
            area += (func.execute(NullFunctionDouble, i) + func.execute(NullFunctionDouble, i + args[2])) / 2;

        return area;
    }
}
