using System;
using OpenCvSharp;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;

public class Class1
{
    public static Matrix<double> FindHomography(List<Point2d> MaybeInliers/*MaybeInlier1*/, List<Point2d> MaybeInliers_dash/*MaybeInlier2*/)
    {
        double[] X = new double[4];
        double[] Y = new double[4];
        double[] X_dash = new double[4];
        double[] Y_dash = new double[4];
        int i = 0;

        for (i = 0; i < 4; i++)
        {
            X[i] = MaybeInliers[i].X;
            Y[i] = MaybeInliers[i].Y;
            X_dash[i] = MaybeInliers_dash[i].X;
            Y_dash[i] = MaybeInliers_dash[i].Y;
        }

        //Ax = B, x is the array of items in homography matrix.

        double[,] Array_A = new double[8, 8]{
            {X[0], Y[0], 1, 0, 0, 0, -X[0]*X_dash[0], -X_dash[0]*Y[0]},
            {0, 0, 0, X[0], Y[0], 1, -X[0]*Y_dash[0], -Y[0]*Y_dash[0]},

            {X[1], Y[1], 1, 0, 0, 0, -X[1]*X_dash[1], -X_dash[1]*Y[1]},
            {0, 0, 0, X[1], Y[1], 1, -X[1]*Y_dash[1], -Y[1]*Y_dash[1]},

            {X[2], Y[2], 1, 0, 0, 0, -X[2]*X_dash[2], -X_dash[2]*Y[2]},
            {0, 0, 0, X[2], Y[2], 1, -X[2]*Y_dash[2], -Y[2]*Y_dash[2]},

            {X[3], Y[3], 1, 0, 0, 0, -X[3]*X_dash[3], -X_dash[3]*Y[3]},
            {0, 0, 0, X[3], Y[3], 1, -X[3]*Y_dash[3], -Y[3]*Y_dash[3]},
        };

        var Matrix_A = Matrix<double>.Build.DenseOfArray(Array_A);

        // matrix B(1*8) 
        double[,] Array_B = new double[1, 8] { { -X_dash[0], -Y_dash[0], -X_dash[1], -Y_dash[1], -X_dash[2], -Y_dash[2], -X_dash[3], -Y_dash[3] } };
        var Matrix_B = Matrix<double>.Build.DenseOfArray(Array_B);

        //calculate H
        Matrix<double> Column_H = Matrix_A.Inverse() * Matrix_B.Transpose();

        double[] Array_H = new double[9];
        Array_H[8] = 1;

        for (int a = 0; a < 8; a++)
        {
            Array_H[a] = Column_H[a, 0];
        }

        var H = Matrix<double>.Build.DenseOfRowMajor(3, 3, Array_H);

        return H;
    }
}
