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

    goodFeaturesToTrack(gray2, corner1, 1500, 0.01, 20);

    if (corner1.empty())
    {
        return status;
    }
    calcOpticalFlowPyrLK(gray2, gray1, corner1, corner2, status, err, Size(10, 10), 4, criteria);
    return status;
}

void Detection::match(Ptr<DescriptorMatcher> matcher, Mat descriptor_object, Mat descriptors_scene, vector<KeyPoint> keypoints_object, vector<KeyPoint> keypoints_scene, vector<DMatch> matches)
{
    matcher->match(descriptor_object, descriptors_scene, matches);

    std::vector<Point2f> obj;
    std::vector<Point2f> scene;
    for (size_t i = 0; i < matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        obj.push_back(keypoints_object[matches[i].queryIdx].pt);
        scene.push_back(keypoints_scene[matches[i].trainIdx].pt);
    }
}

void Detection::flowMotion(Mat& prevImg, Mat& currentImg)
{
    Mat flow(prevImg.size(), CV_32FC2);
    calcOpticalFlowFarneback(prevImg, currentImg, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

    int x = prevImg.cols;
    int y = prevImg.rows;
}

void Detection::tracking(Mat& img1, Mat& img2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)
{
    goodFeaturesToTrack(img1, points1, 1500, 0.01, 20);
	
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);

    calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err, Size(10, 10), 4, criteria);

    int indexCorrection = 0;
    for (int i = 0; i < status.size(); i++)
    {
        Point2f pt = points2.at(i - indexCorrection);
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                status.at(i) = 0;
            }
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}