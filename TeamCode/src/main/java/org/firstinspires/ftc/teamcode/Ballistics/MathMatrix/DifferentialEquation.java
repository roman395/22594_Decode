package org.firstinspires.ftc.teamcode.Ballistics.MathMatrix;

public abstract class DifferentialEquation extends MatrixFunction
{
    // m = (y2 - y1) / (x2 - x1) = d(mx+c)/dx
    /* public OptionalDouble Slope(double x1, double y1, double x2, double y2)
    {
        if (y1 == y2)
            return OptionalDouble.empty();
        return OptionalDouble.of((y2 - y1) / (x2 - x1));
    } */

    private class func_const extends MatrixFunction
    {
        @Override
        public Matrix execute(MatrixFunction func, Matrix... args) // args = { y_prev }
        {
            return args[0];
        }
    }

    // d(y(t))/dx = f(t, y(t))
    abstract public Matrix CauchyFunction(double t, MatrixFunction func);  // f(t, y(t))

    public Matrix MidpointMethod(double total_t, double step, Matrix initial)
    {
        Matrix y_curr = initial, y_prev;

        for (int i = 0; i < total_t / step; i++)
        {
            class func_internal extends MatrixFunction
            {
                private final int i1;
                public func_internal(int i1) { this.i1 = i1; }
                @Override
                public Matrix execute(MatrixFunction func, Matrix... args) // args = { y_prev }
                {
                    func_const y_n = new func_const();
                    return CauchyFunction(2 * i1 + 1, y_n).multiplyByScalar(step / 2).add(args[0]);
                }
            }
            func_internal funcint = new func_internal(i);
            y_prev = y_curr;
            y_curr = CauchyFunction((2 * i + 1) * step / 2, funcint).multiplyByScalar(step).add(y_prev);
        }

        return y_curr;
    }

    @Override
    public Matrix execute(MatrixFunction func, Matrix... args) // args[0].x() = total_t, args[0].y() = step, args[1] = initial
    {
        return MidpointMethod(args[0].x(), args[0].y(), args[1]);
    }
}
