#include "Menu.h"

void Menu::startMenu()
{
	selection = "";
	string defaultSelection = "road1";
	DIR* directory;

	do 
	{
		cout << "Choissisez un dossier : (road1_grey) " << endl;
		cin >> selection;

		if (selection == "\n") selection = defaultSelection;
		
		selection = imgDirectory + selection;

		cout << "Dossier choisi : " << selection << endl;
		directory = opendir(selection.c_str());

		if (directory == NULL)
		{
			cout << "Dossier non existant" << endl;
		}
	} while (directory == NULL);

	transformation.setPath(selection);
	startVideo(directory, selection);

	closedir(directory);
}

void Menu::printImg(vector<Mat> imgs)
{
	int i = 0;
	for(Mat img : imgs)
	{
		printImg(img, i++);
	}
}

void Menu::printImg(Mat img)
{
	printImg(img, 0);
}

void Menu::printImg(Mat img, int idx)
{
	imshow("img_"+idx, img);
}

void Menu::startVideo(DIR* dir, string path)
{
	dirent* entry;
	Mat img;
	Mat prevImg;

	int x = 0;
	int y = 0;

	bool displayMatching = false;
	bool displayDisparity= false;
	bool displayInterestPoint = false;
	bool displayTrajectoryPoints = false;
	bool displayTrajectory = true;

	Mat outImg;
	vector<Mat> outImgs;

	int nbFrame = 0;
	Mat traj = Mat::zeros(600, 600, CV_8UC3);
	
	while (entry = readdir(dir))
	{
		prevImg = img;

		img = transformation.readImgFromPath(path + "/" + entry->d_name, x, y);

		if (img.data && prevImg.data)
		{
			nbFrame++;
			if (displayMatching)
			{
				outImg = transformation.computeMatchings(img, prevImg);
				printImg(outImg);
			}
			if (displayInterestPoint)
			{
				outImgs = transformation.computeInterestingPoints(img, prevImg);
				printImg(outImgs);
			}
			if (displayDisparity)
			{
				outImg = transformation.computeDisparity(img, prevImg);
				printImg(outImg);
			}
			if (displayTrajectoryPoints)
			{
				outImg = transformation.computeTrajectoryPoints(img, prevImg);
				printImg(outImg);
			}
			if (displayTrajectory)
			{
				transformation.drawTrajectory(img, prevImg, nbFrame, traj);
				printImg(img);
			}
			
			char c = waitKey(1);
			if (c == 27)
			{
				return;
			}
		}
	}
}
