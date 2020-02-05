


int RansacHomograhyEstimation(CorspMap* corspMap, CorspMap* inlierMap, CvMat* H) {
    const int num = 4; // minimum number to calc H
    Point2f pt1[num];
    Point2f pt2[num];

    int numOfInliers;
    int totalNumOfPoints = corspMap->num;
    int maxNumOfInliers = 1;

    CorspMap tempInlierMap;

    CvMat* Htmp = cvCreateMat(3, 3, CV_32FC1);

    float p = 0.99f; //how right the estimation could be
    float w = 0.70f; //  𝜔 is the probability of a random data point to be an inlier
    float k = 1000.f; // max iterations
    int iter = 0;

    if( totalNumOfPoints < num*2 ) return -1; // the amount of data is not enough to calc H

    // free memeory operation, not related to RANSAC
    arMalloc(tempInlierMap.mp, MatchPoint, totalNumOfPoints);

    // RANSAC
    inlierMap->num = 0;
    while( iter < k ) {
        // pick 4 corresponding points
        for(int i = 0 ; i < num ; i++ ) {
            int pos;
            pos = rand() % corspMap->num; 
            pt1[i].x = (float)corspMap->mp[pos].x1;
            pt1[i].y = (float)corspMap->mp[pos].y1;
            pt2[i].x = (float)corspMap->mp[pos].x2;
            pt2[i].y = (float)corspMap->mp[pos].y2;
        }

        // compute the homography
        ComputeHomography(pt2, pt1, num, Htmp); // this function has opencv impelentation

        // calculate the distance for each correspondences
        // compute the number of inliers
        tempInlierMap.num = 0;
        CalculateDistance(Htmp, corspMap, &tempInlierMap);

        // choose H with the largest number of inliears
        numOfInliers = tempInlierMap.num;
        if( numOfInliers >= maxNumOfInliers ) {
            maxNumOfInliers = numOfInliers;
            cvCopy(Htmp, H, 0);
            for(int j = 0; j < tempInlierMap.num; j++ ) inlierMap->mp[j] = tempInlierMap.mp[j];
            inlierMap->num = tempInlierMap.num;
        }

        // adaptive algorithm for upodating the necessary iterations
        float inlierProb;
        inlierProb = (float)maxNumOfInliers / (float)totalNumOfPoints;
        if(inlierProb > w) w = inlierProb;
        k = log(1 - p) / log(1 - pow(w, num));
        iter++;
    }

    // release memeory, not related to RANSAC itself
    free(tempInlierMap.mp);
    cvReleaseMat(&Htmp);

   return 0;
}