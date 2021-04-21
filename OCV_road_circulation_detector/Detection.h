#pragma once

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Detection
{
public:
	static void displayMatchings(Mat& image1, Mat& image2, std::vector<Point2f> points1, vector<Point2f> points2, vector<uchar> status1, vector<uchar> status2);
	static vector<uchar> findMatchings(Mat& gray1, Mat& gray2, vector<Point2f>& corner1, vector<Point2f>& corner2);
	static void match(Ptr<DescriptorMatcher> matcher, Mat descriptor_object, Mat descriptors_scene, vector<KeyPoint> keypoints_object, vector<KeyPoint> keypoints_scene, vector<DMatch> matches);
};