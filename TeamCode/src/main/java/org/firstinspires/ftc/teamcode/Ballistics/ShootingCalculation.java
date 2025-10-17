package org.firstinspires.ftc.teamcode.Ballistics;

import static org.firstinspires.ftc.teamcode.Ballistics.Constants.NullFunctionMatrix;

import org.firstinspires.ftc.teamcode.Ballistics.Math.GeneralMath;
import org.firstinspires.ftc.teamcode.Ballistics.Math.Vector;
import org.firstinspires.ftc.teamcode.Ballistics.MathDouble.Function;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.DifferentialEquation;
import org.firstinspires.ftc.teamcode.Ballistics.MathDouble.Integral;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.Matrix;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.MatrixFunction;

import java.util.OptionalDouble;

public class ShootingCalculation
{
    private static class BallisticEquation extends DifferentialEquation
    {
        @Override
        public Matrix CauchyFunction(double t, MatrixFunction func)
        {
            Matrix retval = new Matrix(3);

            retval.element(0, 0, OptionalDouble.of(func.execute(NullFunctionMatrix, new Matrix(3, t)).Size() * func.executeIndexed(0, 0, NullFunctionMatrix, new Matrix(t)) * Constants.DragBigConstant));
            retval.element(0, 1, OptionalDouble.of(func.execute(NullFunctionMatrix, new Matrix(3, t)).Size() * func.executeIndexed(0, 1, NullFunctionMatrix, new Matrix(t)) * Constants.DragBigConstant));
            retval.element(0, 2, OptionalDouble.of(func.execute(NullFunctionMatrix, new Matrix(3, t)).Size() * func.executeIndexed(0, 2, NullFunctionMatrix, new Matrix(t)) * Constants.DragBigConstant - Constants.g));

            return retval;
        }
    }

    private static class F extends MatrixFunction
    {
        @Override
        public Matrix execute(MatrixFunction func, Matrix... args) // args[0].x() = t_total, args[1] = v0, args[2](x = x1, y = y1, z = z1, w = a, v = b) = freeCoefficients
        {
            Integral integrate = new Integral();
            BallisticEquation ballisticEq = new BallisticEquation();
            Matrix retval = new Matrix(5);

            // Step 1: Solve SODE
            final Matrix v0 = args[1];

            class FuncX extends Function
            {
                @Override
                public double execute(Function func, double... args)
                {
                    return ballisticEq.executeIndexed(0, 0, NullFunctionMatrix, new Matrix(args[0], Constants.Step), v0);
                }
            }

            class FuncY extends Function
            {
                @Override
                public double execute(Function func, double... args)
                {
                    return ballisticEq.executeIndexed(0, 0, NullFunctionMatrix, new Matrix(args[0], Constants.Step), v0);
                }
            }

            class FuncZ extends Function
            {
                @Override
                public double execute(Function func, double... args)
                {
                    return ballisticEq.executeIndexed(0, 0, NullFunctionMatrix, new Matrix(args[0], Constants.Step), v0);
                }
            }

            // Step 2: Integrate function
            retval.element(0, 0, OptionalDouble.of(integrate.execute(new FuncX(), 0, args[0].x())));
            retval.element(0, 1, OptionalDouble.of(integrate.execute(new FuncY(), 0, args[0].x())));
            retval.element(0, 2, OptionalDouble.of(integrate.execute(new FuncZ(), 0, args[0].x())));

            retval.element(0, 3, OptionalDouble.of(integrate.execute(new FuncY(), 0, args[0].x())));
            retval.element(0, 4, OptionalDouble.of(integrate.execute(new FuncZ(), 0, args[0].x())));

            // Step 3: multiply (0, 4) by (1 - a)
            retval = new Matrix(retval.x(), retval.y(), retval.z(), (1 - args[2].w()) * retval.w(), retval.v());

            // Step 4: Add free coefficients
            final Matrix freeCoefficients = new Matrix(-args[2].x(), -args[2].y(), -args[2].z(), -args[2].v(), -(Constants.Height + Constants.HeightError + Constants.BallRadius));
            retval = retval.add(freeCoefficients);

            // Return
            return retval;
        }
    }

    public static Vector GetVelocity(double[] robotVelocity, double[] landingSpot, double distToGoal, double wallSlopeAngle) // x is codirectional to the direction that the camera is pointing in, z is vertical
    {
        F func1 = new F();
        class Fun extends MatrixFunction
        {
            @Override
            public Matrix execute(MatrixFunction func, Matrix... args) // args[0].contents = { t_total, v_0x, v_0y, v_0z, t_height }
            {
                return func1.execute(NullFunctionMatrix, args[0], new Matrix(args[0].y(), args[0].z(), args[0].w()), new Matrix(landingSpot[0], landingSpot[1], 0, Math.tan(wallSlopeAngle), Math.sqrt(distToGoal * distToGoal * Math.sin(wallSlopeAngle) * Math.cos(wallSlopeAngle))));
            }
        }
        Fun func = new Fun();

        // Get a guess (= calculation without drag)
        final double tga = landingSpot[0] / landingSpot[1];
        final double totDist = Math.sqrt(landingSpot[0] * landingSpot[0] + landingSpot[1] * landingSpot[1]);
        final double m = totDist - distToGoal;
        final double i = Constants.Height + Constants.HeightError + Constants.BallRadius;

        final double alpha_guess = Math.atan(i * totDist / (distToGoal * m));
        final double v0_guess = Math.sqrt(Constants.g * totDist * (1 + Math.tan(alpha_guess)) / 2 * tga);
        final double ttotal_guess = 2 * v0_guess * Math.sin(alpha_guess);
        final double theight_guess = distToGoal / (v0_guess * Math.cos(alpha_guess));
        Matrix guessSpeed = new Matrix(v0_guess * Math.cos(alpha_guess), 0, v0_guess * Math.sin(alpha_guess));

        double angleRot = Math.atan(tga);
        final double[][] rotmat = { { Math.cos(-angleRot), -Math.sin(-angleRot), 0 },
                                    { Math.sin(-angleRot),  Math.cos(-angleRot), 0 },
                                    { 0                  ,  0                  , 1 } };
        final Matrix rot = new Matrix(3, 3, rotmat);
        guessSpeed = rot.multiply(guessSpeed);
        final Matrix guess = new Matrix(ttotal_guess, guessSpeed.x(), guessSpeed.y(), guessSpeed.z(), theight_guess);

        // Solve SOE
        Matrix solution = GeneralMath.NewtonSOE(func, Constants.Precision, 65535, guess, 5, Constants.Step);
        Matrix robspeed = new Matrix(robotVelocity[0], robotVelocity[1], robotVelocity[2]);
        solution = solution.subtract(robspeed);

        // Convert to vector

        return Vector.XYZ(solution.y(), solution.z(), solution.w());
    }
}
