package org.firstinspires.ftc.teamcode.Ballistics.MathMatrix;

import org.firstinspires.ftc.teamcode.Ballistics.Math.GeneralMath;

import java.util.OptionalDouble;

public class Matrix
{
    private final int rows, columns;
    public double[][] contents;

    // Constructors
    public Matrix(int rows, int columns, double[]... contents)
    {
        this.rows = rows;
        this.columns = columns;
        this.contents = contents;
    }

    public Matrix(int rows, int columns)
    {
        this.rows = rows;
        this.columns = columns;
        contents = new double[rows][columns];
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                this.contents[j][i] = 0;
    }

    // Vectors
    public Matrix(int rows, double... contents)
    {
        this.rows = rows;
        this.columns = 1;
        this.contents = new double[][]{contents};
    }

    public Matrix(int rows)
    {
        this.rows = rows;
        this.columns = 1;
        contents = new double[rows][columns];
        for (int i = 0; i < rows; i++)
            this.contents[0][i] = 0;
    }

    public Matrix(double... contents)
    {
        this.rows = contents.length;
        this.columns = 1;
        this.contents = new double[][]{contents};
    }
    // ---/-/---
    // ---/-/---

    // Operators
    public double x()
    {
        return contents[0][0];
    }
    public double y()
    {
        return contents[0][1];
    }
    public double z()
    {
        return contents[0][2];
    }
    public double w()
    {
        return contents[0][3];
    }
    public double v()
    {
        return contents[0][4];
    }
    public double u()
    {
        return contents[0][5];
    }

    public double element(int x, int y, OptionalDouble set)
    {
        if (set.isPresent())
            contents[y][x] = set.getAsDouble();
        return contents[y][x];
    }

    /*public int rows()
    {
        return rows;
    }

    public int columns()
    {
        return columns;
    }*/

    public int width()
    {
        return rows;
    }// duplicate methods??? nikitos ti che dolbayob
    public int height()
    {
        return columns;
    }

    public Matrix delRow(int row)
    {
        Matrix retval = new Matrix(rows - 1, columns);

        System.arraycopy(contents, 0, retval.contents, 0, row);
        System.arraycopy(contents, row + 1, retval.contents, row, contents.length - row - 1);

        return retval;
    }

    public Matrix delColumn(int column)
    {
        Matrix retval = new Matrix(rows, columns - 1);

        for (int i = 0; i < contents.length; i++)
        {
            System.arraycopy(contents[i], 0, retval.contents[i], 0, column);
            System.arraycopy(contents[i], column + 1, retval.contents[i], column, contents[i].length - column - 1);
        }

        return retval;
    }

    public double algebraicAdjugate(int x, int y)
    {
        Matrix minor = this.delRow(x);
        minor = minor.delColumn(y);

        return ((x + y) % 2 == 0 ? 1 : -1) * (minor.det());
    }



    public Matrix add(Matrix c1)
    {
        Matrix retval = new Matrix(this.rows, this.columns);

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                retval.contents[i][j] = this.contents[i][j] + c1.contents[i][j];

        return retval;
    }

    public Matrix negate()
    {
        Matrix retval = new Matrix(this.rows, this.columns);

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                retval.contents[i][j] = -this.contents[i][j];

        return retval;
    }

    public Matrix subtract(Matrix c1)
    {
        return this.add(c1.negate());
    }

    public Matrix multiplyByScalar(double c1)
    {
        Matrix retval = new Matrix(this.rows, this.columns);

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                retval.contents[i][j] = this.contents[i][j] * c1;

        return retval;
    }

    public Matrix multiply(Matrix c1)
    {
        final Matrix retval = new Matrix(this.rows, c1.columns);
        int k;
        double sum = 0;

        for (int i = 0; i < this.rows; i++)
            for (int j = 0; j < c1.columns; j++)
            {
                for (k = 0, sum = 0; k < this.columns; k++)
                    sum += this.element(i, k ,OptionalDouble.empty()) * c1.element(k, j, OptionalDouble.empty());
                retval.element(i, j, OptionalDouble.of(sum));
            }

        return retval;
    }

    public Matrix divideByScalar(double c1)
    {
        Matrix retval = new Matrix(this.rows, this.columns);

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                retval.contents[i][j] = this.contents[i][j] / c1;

        return retval;
    }

    public double Size()
    {
        double retval = 0;

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                retval += contents[j][i] * contents[j][i];

        return Math.sqrt(retval);
    }

    public double det()
    {
        return GeneralMath.getDet(contents, contents.length);
    }

    public Matrix adj()
    {
        Matrix retval = new Matrix(rows, columns);

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < columns; j++)
                retval.element(i, j, OptionalDouble.of(this.element(i, j, OptionalDouble.empty())));

        return retval;
    }

    public Matrix inv()
    {
        return adj().divideByScalar(det());
    }
    // ---/-/---
}
