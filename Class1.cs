using System;

public class Class1
{
	public Class1()
	{
        var H = Matrix<double>.Build.Dense(3, 3);
        var maybeInlier1 = new List<Point2d>();
        var maybeInlier2 = new List<Point2d>();

        var random = new Random();
        int[] IndexOfMaybeInlier = new int[4];
        int i = 0;



        //random select four pair of points
        while (i < 4)
        {
            int index = random.Next(betterKp1_tmp.Count);
            if (!maybeInlier1.Contains(betterKp1_tmp[index]))
            {
                maybeInlier1.Add(betterKp1_tmp[index]);
                maybeInlier2.Add(betterKp2_tmp[index]);

                IndexOfMaybeInlier[i] = index;
                i++;
            }
        }
        //--------------------------------------------------


        H = FindHomography(maybeInlier1, maybeInlier2);



        System.Diagnostics.Debug.WriteLine("the Homography matrix is:" + H);


    }
}
