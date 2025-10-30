package org.firstinspires.ftc.teamcode.Ballistics.Math;

import static org.firstinspires.ftc.teamcode.Ballistics.Constants.NullFunctionMatrix;

import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.Matrix;
import org.firstinspires.ftc.teamcode.Ballistics.MathMatrix.MatrixFunction;

public class GeneralMath
{
    public static double multiply(double[] arr)
    {
        double retval = 1;

        for (int i = 0; i < arr.length; i++)
            retval *= arr[i];

        return retval;
    }

    public static double PartialDerivative(MatrixFunction func, Matrix x, double step, int var, int varret)
    {
        Matrix matrStep = x;
        matrStep.contents[0][var] += step;

        return (func.executeIndexed(varret, 0, NullFunctionMatrix, matrStep) - func.executeIndexed(0, 0, NullFunctionMatrix, x)) / step;
    }

    public static double getDet(double[][] mat, int n) {

        // Base case: if the matrix is 1x1
        if (n == 1) {
            return mat[0][0];
        }

        // Base case for 2x2 matrix
        if (n == 2) {
            return mat[0][0] * mat[1][1] -
                    mat[0][1] * mat[1][0];
        }

        // Recursive case for larger matrices
        double res = 0;
        for (int col = 0; col < n; ++col) {

            // Create a submatrix by removing the first
            // row and the current column
            double[][] sub = new double[n - 1][n - 1];
            for (int i = 1; i < n; ++i) {
                int subcol = 0;
                for (int j = 0; j < n; ++j) {

                    // Skip the current column
                    if (j == col) continue;

                    // Fill the submatrix
                    sub[i - 1][subcol++] = mat[i][j];
                }
            }

            // Cofactor expansion
            int sign = (col % 2 == 0) ? 1 : -1;
            res += sign * mat[0][col] * getDet(sub, n - 1);
        }

        return res;
    }

    public static Matrix NewtonSOE(MatrixFunction F, double precision, double max, Matrix init_guess, int N, double derivativeStep) // F(x) = 0
    {
        Matrix retval = init_guess, Jacobian = new Matrix(N, N), prev = init_guess, Wn1;

        for (int k = 0; k < max && retval.subtract(prev).Size() > precision; k++, prev = retval)
        {
            for (int i = 0; i < N; i++) // calculating the Jacobian matrix
                for (int j = 0; j < N; j++)
                {
                    Jacobian.contents[j][i] = PartialDerivative(F, retval, derivativeStep, i, j);
                }
            Wn1 = Jacobian.inv();

            retval = prev.subtract(Wn1.multiply(F.execute(NullFunctionMatrix, prev)));
        }

        return retval;
    }
}
