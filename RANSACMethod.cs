using System;

public class Class1
{
    public static RansacMethod(List<Point2d> kp1, List<Point2d> kp2, out List<Point2d> betterkp1, out List<Point2d> betterkp2)
    {
        int n = 4;
        int iteration = 0;
        int d = 0;
        var alsoInlier1 = new List<Point2d>();
        var alsoInlier2 = new List<Point2d>();
        var bestError = int.MaxValue;
        var Inlier = new List<Point2d>();


        double k = 1000.0f;

        //iteration
        while (iteration < k)
        {
            var maybeInlier1 = new List<Point2d>();
            var maybeInlier2 = new List<Point2d>();

            var random = new Random();
            int[] IndexOfMaybeInlier = new int[4];
            int i = 0;



            //random select four pair of points
            while (i < n)
            {
                int index = random.Next(kp1.Count);
                if (!maybeInlier1.Contains(kp1[index]))
                {
                    maybeInlier1.Add(kp1[index]);
                    maybeInlier2.Add(kp2[index]);

                    IndexOfMaybeInlier[i] = index;
                    i++;
                }
            }

            Array.Sort(IndexOfMaybeInlier);

            //calculate model(homography) by using the random select points 
            Matrix<double> H = FindHomography(maybeInlier1, maybeInlier2);

            //get the rest of the data. 
            var restData1 = new List<Point2d>(kp1.ToArray());
            var restData2 = new List<Point2d>(kp2.ToArray());


            for (int j = restData1.Count - 1; j >= 0; j--)
            {
                foreach (int item in IndexOfMaybeInlier)
                {
                    restData1.RemoveAt(item);
                    restData2.RemoveAt(item);
                }
            }

            //calculate the rest data, if it fits the model, add it to alsoInlier.
            for (int m = 0; m < restData1.Count; m++)
            {
                double t = CalculateDistance(restData1[m], restData2[m], H);
                if (t < 2.0f)
                {
                    alsoInlier1.Add(restData1[m]);
                    alsoInlier2.Add(restData2[m]);
                }
            }

            // end if We founnd enough number of inlier.


        }








    }
}
