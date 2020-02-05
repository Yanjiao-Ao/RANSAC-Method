using OpenCvSharp;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

//--------------------------------------------------------------------------------------------------

class calcR
{

    //--------------------------------------------------------------------------------------------------

    /**
     * Calculate the Rotation matrix from A->B: B = R*A
     */
    public static Matrix<double> CalcRotation(Matrix<double> A, Matrix<double> B)
    {
        
        var M = Matrix<double>.Build;
        var H = A * B.Transpose();
        var svd = H.Svd();
        var tmp = svd.U * svd.VT;//add() or not???
        var e = M.DenseOfDiagonalArray(new[] { 1, 1, tmp.Determinant() });
        var R = svd.VT.Transpose() * e * svd.U.Transpose();
        return R;
    }

    //--------------------------------------------------------------------------------------------------
    /*
     * Homography: Calculate the H matirx.
     * H = A-1*B
     */

    public static Matrix<double> FindHomography(List<Point2d> points_src, List<Point2d> points_dst)
    {
        // Check if the two points are of same size and the size the larger than 4
        Debug.Assert(points_src.Count == points_dst.Count);
        Debug.Assert(points_src.Count >= 4);

        // Ax = B, x is the array of items in homography matrix.
        var A = Matrix<double>.Build.Dense(2 * points_src.Count, 8);
        for (int i = 0; i < points_src.Count; i += 2)
        {
            double u = points_dst[i].X;
            double v = points_dst[i].Y;
            double x = points_src[i].X;
            double y = points_src[i].Y;
            double[,] row1 = new double[1, 8] { { -x, -y, -1, 0, 0, 0, u * x, u * y } };
            var row_1 = Matrix<double>.Build.DenseOfArray(row1);
            double[,] row2 = new double[1, 8] { { 0, 0, 0, -x, -y, -1, v * x, v * y } };
            var row_2 = Matrix<double>.Build.DenseOfArray(row2);
            A.SetSubMatrix(i, 0, row_1);
            A.SetSubMatrix(i + 1, 0, row_2);
        }

        var B = Matrix<double>.Build.Dense(2 * points_src.Count, 1);
        for (int i = 0; i < points_src.Count; i += 2)
        {
            B[i, 0] = -1 * points_dst[i].X;
            B[i + 1, 0] = -1 * points_dst[i].Y;
        }

        // Solve Ax = B with least square
        var h = A.Solve(B);

        var H = Matrix<double>.Build.Dense(3, 3);
        H[0, 0] = h[0, 0]; H[0, 1] = h[1, 0]; H[0, 2] = h[2, 0];
        H[1, 0] = h[3, 0]; H[1, 1] = h[4, 0]; H[1, 2] = h[5, 0];
        H[2, 0] = h[6, 0]; H[2, 1] = h[7, 0]; H[2, 2] = 1;

        return H;
    }

    //---------------------------------------------------------------------------------------------------------------

    public static double CalculateDistance(Point2d kp1, Point2d kp2, Matrix<double> H) {
        var c1 = Vector<double>.Build.DenseOfArray(new[] { kp1.X, kp1.Y, 1 });
        var c2 = Vector<double>.Build.DenseOfArray(new[] { kp2.X, kp2.Y, 1 });

        var transformed_c1 = H * c1;
        var err_vec = transformed_c1 - c2;

        double dis = err_vec[0] * err_vec[0] + err_vec[1] * err_vec[1];

        return dis;
       
    }

    //----------------------------------------------------------------------------------------------------------------

    //public static Tuple<List<Point2d>, List<Point2d>> RansacMethod(List<Point2d> kp1, List<Point2d> kp2) {

    //    var betterkp1 = new List<Point2d>();
    //    var betterkp2 = new List<Point2d>();
    //    var betterkpTuple = new Tuple<List<Point2d>, List<Point2d>>(betterkp1, betterkp2);

    //    int maxNumberOfInlier = 4;
    //    double k = 1000.0f;
    //    double w = 0.70f;
    //    double p = 0.99f;
    //    int num = 10;
    //    double max_err = 0; // threshold to judge whether other points are inliners or not

    //    //iteration
    //    int iteration = 0;
    //    while (iteration < k)
    //    {
    //        var maybeInlier1 = new List<Point2d>();
    //        var maybeInlier2 = new List<Point2d>();
    //        var alsoInlier1 = new List<Point2d>();
    //        var alsoInlier2 = new List<Point2d>();

    //        //random select four pair of points
    //        var random = new Random();
    //        for (int i = 0; i < num; i++)
    //        {
    //            int index = random.Next(kp1.Count);
    //            maybeInlier1.Add(kp1[index]);
    //            maybeInlier2.Add(kp2[index]);
    //        }

    //        //calculate model(homography) by using the random select points 
    //        var H = FindHomography(maybeInlier1, maybeInlier2);

    //        //calculate the data, if it fits the model, add it to alsoInlier.
    //        for (int i=0; i<num; i++) {
    //            double err = CalculateDistance(kp1[i], kp2[i], H);
    //            if(err > max_err) {
    //                max_err = err;
    //            }
    //        }
    //        Console.WriteLine("err is " + max_err);
    //        for (int i = 0; i < kp1.Count; ++i) {
    //            double err = CalculateDistance(kp1[i], kp2[i], H);
    //            if (err < max_err) {
    //                alsoInlier1.Add(kp1[i]);
    //                alsoInlier2.Add(kp2[i]);
    //            }
    //        }

    //        // end if We founnd enough number of inlier.
    //        if (alsoInlier1.Count >= maxNumberOfInlier) {
    //            maxNumberOfInlier = alsoInlier1.Count;
    //            betterkp1 = alsoInlier1;
    //            betterkp2 = alsoInlier2;
    //        }

    //        //k is dymatic.
    //        double inlierProb = maxNumberOfInlier / kp1.Count;
    //        if (inlierProb > w) w = inlierProb;
    //        k = Math.Log(1 - p) / Math.Log(1 - Math.Pow(w, 4));
    //        iteration++;
    //    }
    //    return betterkpTuple;
    //}



    public static Tuple<List<Point2d>, List<Point2d>> RansacMethod(List<Point2d> kp1, List<Point2d> kp2) {
        
        var betterkp1 = new List<Point2d>();
        var betterkp2 = new List<Point2d>();
        var betterkpTuple = new Tuple<List<Point2d>, List<Point2d>>(betterkp1, betterkp2);

        double error_thred = 4.0f;
        int num = 10;
        int maxNumberOfInlier = num;
        double k = 100.0f;
        double w = 0.70f;
        double p = 0.99f;

        //iteration
        int iteration = 0;
        while (iteration < k) {
            var maybeInlier1 = new List<Point2d>();
            var maybeInlier2 = new List<Point2d>();
            var alsoInlier1 = new List<Point2d>();
            var alsoInlier2 = new List<Point2d>();

            //random select four pair of points
            var random = new Random();
            int[] IndexOfMaybeInlier = new int[num];
            for (int i=0; i< num; i++) { 
                int index = random.Next(kp1.Count);
                if (!maybeInlier1.Contains(kp1[index])) {
                    maybeInlier1.Add(kp1[index]);
                    maybeInlier2.Add(kp2[index]);
                }
            }
            Array.Sort(IndexOfMaybeInlier);

            //calculate model(homography) by using the random select points 
            Matrix<double> H = FindHomography(maybeInlier1, maybeInlier2);

            //calculate the rest data, if it fits the model, add it to alsoInlier.
            for (int i = 0; i < kp1.Count; i++) {
                if (IndexOfMaybeInlier.Contains(i))
                    continue;
                double err = CalculateDistance(kp1[i], kp2[i], H);
                if (err < error_thred) {
                    alsoInlier1.Add(kp1[i]);
                    alsoInlier2.Add(kp2[i]);
                }
            }

            // end if We found enough number of inlier.
            var numberOfInlier = alsoInlier1.Count + maybeInlier1.Count;
            if (numberOfInlier >= maxNumberOfInlier) {
                maxNumberOfInlier = numberOfInlier;
                betterkp1.Clear();
                betterkp2.Clear(); 
                betterkp1.AddRange(maybeInlier1);
                betterkp2.AddRange(maybeInlier2);
                betterkp1.AddRange(alsoInlier1);
                betterkp2.AddRange(alsoInlier2);
            }

            // k is dymatic.
            double inlierProb = maxNumberOfInlier / kp1.Count;
            if (inlierProb > w) w = inlierProb;
            k = Math.Log(1 - p) / Math.Log(1 - Math.Pow(w, num));
            iteration++;

        }

        return betterkpTuple;
    }

    //--------------------------------------------------------------------------------------------------

    /**
     * Project from 2d image coordinate to 3d camera coordinate
     */
    public static Vector<double> From2dTo3d(Point2f p, int W, int H)
    {
        double m = p.Y;
        double n = p.X;
        double u = (m + 0.5) / W;
        double v = (n + 0.5) / H;
        double alpha = (u - 0.5) * 2 * Math.PI;
        double beta = (0.5 - v) * Math.PI;
        double X = Math.Cos(beta) * Math.Sin(alpha);
        double Y = Math.Sin(beta);
        double Z = Math.Cos(beta) * Math.Cos(alpha);
        var V = Vector<double>.Build;
        var ret = V.DenseOfArray(new[] { X, Y, Z });
        return ret;
    }

    //-----------------------------------------------------------------------------------------------------
    /*
     *Rotate images 90 degrees
     */
    public static void RotateAndResize(Mat src, out Mat dest, bool isRight/*Left is Basic*/)
    {
        dest = new Mat();

        var center = new Point2f(src.Cols / 2, src.Cols / 2);

        Mat rotationMat = Cv2.GetRotationMatrix2D(center, 90, 1);
        Cv2.WarpAffine(src, dest, rotationMat, new Size(src.Rows, src.Cols));

        if (isRight)
        {
            Cv2.Flip(dest, dest, FlipMode.XY);
        }
    }


    //--------------------------------------------------------------------------------------------------

    static void Main()
    {
        //Mat src = new Mat("/Users/feiran-l/Desktop/ConsoleApp01/1.jpeg", ImreadModes.Grayscale);
        //Mat dst = new Mat("/Users/feiran-l/Desktop/ConsoleApp01/2.jpeg", ImreadModes.Grayscale);

        Mat src = new Mat("/Users/yanjiao-a/Desktop/ransactest/1.jpeg");
        Mat dst = new Mat("/Users/yanjiao-a/Desktop/ransactest/2.jpeg");

        //Compress the original image
        //var rSrc = new Mat();
        //var rDst = new Mat();
        //if (oriSrc.Cols > 420 && oriSrc.Rows > 600)
        //{
        //    Cv2.Resize(oriSrc, rSrc, new Size(420, 600));
        //    Cv2.Resize(oriDst, rDst, new Size(420, 600));
        //}
        //else
        //{
        //    rSrc = oriSrc;
        //    rDst = oriDst;
        //}

        ////Rotate the image 90 degree 

        //RotateAndResize(rSrc, out Mat src, false);
        //RotateAndResize(rDst, out Mat dst, false);


        // Step1: Detect the keypoints and generate their descriptors using SURF
        ORB orb = ORB.Create();
        KeyPoint[] kp1, kp2;
        Mat desc1 = new Mat();
        Mat desc2 = new Mat();
        orb.DetectAndCompute(src, null, out kp1, desc1);
        orb.DetectAndCompute(dst, null, out kp2, desc2);

        // Step2: Matching descriptor vectors with a brute force matcher
        var bfMatcher = new BFMatcher();
        var matches = bfMatcher.KnnMatch(desc1, desc2, k: 2);

        // Step3: Ratio test for outlier removal
        var betterKp1 = new List<Point2f>();
        var betterKp2 = new List<Point2f>();
        var betterMatches = new List<DMatch>();
        foreach (DMatch[] items in matches)
        {
            if (items[0].Distance < 0.75 * items[1].Distance)
            {
                betterKp1.Add(kp1[items[0].QueryIdx].Pt);
                betterKp2.Add(kp2[items[0].TrainIdx].Pt);
                betterMatches.Add(items[0]);
            }
        }
        //***Draw matches after ratio test
        var ratioMat = new Mat();
        Cv2.DrawMatches(src, kp1, dst, kp2, betterMatches, ratioMat);

        // Step4: RANSAC for outlier removal
        var bestKp1 = new List<Point2f>();
        var bestKp2 = new List<Point2f>();
        Point2d Point2fToPoint2d(Point2f pf) => new Point2d(((double)pf.X), ((double)pf.Y));
        var betterKp1_tmp = betterKp1.ConvertAll(Point2fToPoint2d);
        var betterKp2_tmp = betterKp2.ConvertAll(Point2fToPoint2d);
        var output = new Mat();

        //-------------------------------------
        // use my RANSAC to calculate
        var bestTuple = RansacMethod(betterKp1_tmp, betterKp2_tmp);

        // test homography
        // var H = FindHomography(betterKp1_tmp, betterKp2_tmp);
        var H = FindHomography(bestTuple.Item1, bestTuple.Item2);
        Mat H_mat = new Mat(new Size(3, 3), MatType.CV_64FC1);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                H_mat.Set<double>(i, j, H[i, j]);
            }
        }
        Cv2.WarpPerspective(src, src, H_mat, src.Size());

        // show
        Mat plot_img = new Mat(new Size(src.Width, src.Height + dst.Height), MatType.CV_8UC3);
        plot_img.SetTo(new Scalar(0,0,0));
        var tmp1 = new Mat(plot_img, new Rect(0, 0, src.Width, src.Height));
        var tmp2 = new Mat(plot_img, new Rect(0, src.Height-1, dst.Width, dst.Height));
        src.CopyTo(tmp1);
        dst.CopyTo(tmp2);
        Cv2.ImShow("plot", plot_img);
        Cv2.WaitKey();
        Cv2.ImWrite("/Users/yanjiao-a/Desktop/ransactest/ret.png", plot_img);
        //-------------------------------------

    }
}

