package org.firstinspires.ftc.teamcode.Ballistics.Math;

public class Vector
{
    public double alpha, beta, radius;

    public Vector(double alpha, double beta, double radius)
    {
        this.alpha = alpha;
        this.beta = beta;
        this.radius = radius;
    }

    private static double XYZToAbsolute(double x, double y, double z)
    {
        return Math.sqrt(x * x + y * y + z * z);
    }

    private static double XYZToBeta(double x, double y, double z)
    {
        return Math.acos(x * x / (y + x * x));
    }

    private static double XYZToAlpha(double x, double y, double z)
    {
        return Math.acos(y + x * x / (XYZToAbsolute(x, y, z) * x));
    }

    public static Vector XYZ(double x, double y, double z)
    {
        return new Vector(XYZToAlpha(x, y, z), XYZToBeta(x, y, z), XYZToAbsolute(x, y, z));
    }
}
