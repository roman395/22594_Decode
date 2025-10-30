package org.firstinspires.ftc.teamcode.Ballistics.MathMatrix;

public class NullFuncMatrix extends MatrixFunction
{
    @Override
    public Matrix execute(MatrixFunction func, Matrix... args) {
        return new Matrix(1);
    }
}
