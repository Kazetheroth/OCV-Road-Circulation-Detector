#include "Menu.h"
#include "Constants.h"

void Menu::startMenu()
{
	string selection = "";
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
	bool displayDisparity= true;
	bool displayInterestPoint = false;

	Mat outImg;
	vector<Mat> outImgs;
	
	while (entry = readdir(dir))
	{
		prevImg = img;

		img = transformation.readImgFromPath(path + "/" + entry->d_name, x, y);

		if (img.data && prevImg.data)
		{
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
			
			char c = waitKey(0);
			if (c == 27)
			{
				return;
			}
		}
	}
}
