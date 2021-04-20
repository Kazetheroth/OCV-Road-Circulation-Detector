#include "Detection.h"

void Detection::displayMatchings(Mat& image1, Mat& image2, vector<Point2f> points1, vector<Point2f> points2, vector<uchar> status1, vector<uchar> status2)
{
    int nbMatchingA = 0, nbMatchingB = 0;
    for (int i = 0; i < status1.size(); i++)
    {
        if (i >= points1.size())
        {
            break;
        }
    	
        if (status1[i] > 0)
        {
            circle(image1, Point(points1[i].x, points1[i].y), 4, Scalar(0, 255, 0), -1, 8, 0);
            nbMatchingA++;
        }
    }
    for (int i = 0; i < status2.size(); i++)
    {
		if (i >= points2.size())
        {
            break;
        }

    	if (status2[i] > 0)
        {
            circle(image2, Point(points2[i].x, points2[i].y), 4, Scalar(0, 255, 0), -1, 8, 0);
            nbMatchingB++;
        }
    }
}

vector<uchar> Detection::findMatchings(Mat& gray1, Mat& gray2, vector<Point2f>& corner1, vector<Point2f>& corner2)
{
    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);

    goodFeaturesToTrack(gray1, corner1, 1500, 0.01, 20);

    if (corner1.empty())
    {
        return status;
    }
    calcOpticalFlowPyrLK(gray2, gray1, corner1, corner2, status, err, Size(10, 10), 4, criteria);
    return status;
}