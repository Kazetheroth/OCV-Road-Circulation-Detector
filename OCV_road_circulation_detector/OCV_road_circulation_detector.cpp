// OCV_road_circulation_detector.cpp : Ce fichier contient la fonction 'main'. L'exécution du programme commence et se termine à cet endroit.
//

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
	Mat img = imread("../img/charlie.jpg");
    namedWindow("image", WINDOW_NORMAL);
    imshow("image", img);
    waitKey(0);
    return 0;
}
