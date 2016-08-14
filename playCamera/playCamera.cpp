// playCamera.cpp : Defines the entry point for the console application.
//

/*
TODO:
Distinguish table and chair based on size
*/

#define SAVE
#define LIGHT
//#define TIMER

#include "stdafx.h"
#include "helper.h"
#include "CmFile.h"
#include "SoundController.h"
#include "VideoRecorder.h"
#include "MultithreadCam.h"
#include "RectFinder.h"

int _tmain(int argc, _TCHAR* argv[])
{
	MultithreadCam multicam();
#ifdef SAVE
	VideoRecorder recorder(&multicam);
#endif

#ifdef LIGHT
	CURL* curl;
	if (!initCurl(curl))
	{
		cout << "Cannot control lighting" << endl;
		return -1;
	}
#endif
	
	SoundController sound_controller;

	vector<RoomPhotos> vecRP = prepareData();
	
	cv::Mat lastFrame, lastStableFrame;
	bool isProcessed = false;
	int numUnchanged = 0;

	Room lastLighting = OFF;

    // Retrieve an image
	cv::Mat NowFrame;
	cv::Mat background;
	
	cv::Mat bg_mask;
	cv::Ptr<cv::BackgroundSubtractor> p_mog2 = cv::createBackgroundSubtractorMOG2();	

	const int MIN_UNCHANGED = 15;
	const float DIFF_THRESH = 3.0;
	vector<cv::Mat> stableFrames(MIN_UNCHANGED);
	cv::Mat prevFrame;

    RectFinder chair_finder(CHAIR_COLORS, cv::Scalar(255, 0, 0));
    RectFinder table_finder(TABLE_COLORS, cv::Scalar(0, 0, 255));



	/*********************************************/
	//SUMMER EDIT HERE
	Room current_room = OFF;//STUDY DINING LIVING
	/***********************************************/
	//setLight(curl, current_room);
	sound_controller.setRoom(current_room);
	while(true)
	{
		setLight(curl, current_room);
		Sleep(2000);
	}

	/*
	cv::Scalar COLORS1 = cv::Scalar(0, 40, 0);
	cv::Scalar COLORS2 = cv::Scalar(30, 70, 50);
	tuple<cv::Scalar, cv::Scalar> COLORS = tuple<cv::Scalar, cv::Scalar>(COLORS1, COLORS2);
	while(true)
	{
		cv::Mat f = multicam.getImage();
		cv::cvtColor(f, f, CV_RGB2BGR);
		cv::Scalar mean = cv::mean(f,  getROIMask());
		cv::Mat rectFrame = drawColorRects(findColor(f, COLORS, cv::Scalar(255, 255, 255)), f.size());
		cv::waitKey(100);
	}
	*/

	while (true)
    {
		int hour = getCurrentHour();
		cout << hour << endl;
#ifdef TIMER
		if (hour < 18 || hour >= 22)
		{
#ifdef LIGHT
			if (hour >= 7 && hour < 18) setLight(curl, OFF);
			else if (hour >= 22 || hour < 7) setLight(curl, ON);
#endif
			sound_controller.setRoom(OFF);
			cout << "Disable detection!" << endl;
			Sleep(1000 * 60); //wait 1 min
			continue;
		}
		else
		{
			setLight(curl, ON);
			cout << "Detecting!" << endl;
			Sleep(1000 * 60);
			continue;
		}
#endif
		NowFrame = multicam.getImage();
        chair_finder.process(NowFrame, lastLighting);
        table_finder.process(NowFrame, lastLighting);
        cv::Mat combinedFrame = chair_finder.drawColorRects();
        combinedFrame = table_finder.drawColorRects(combinedFrame);

		show("combined", combinedFrame);

		cv::Mat grayFrame;
		cv::cvtColor(combinedFrame, grayFrame, CV_BGR2GRAY);
        //TODO: Check if this is necessary
		if (cv::countNonZero(grayFrame) > 0)
		{
			if (!lastFrame.empty())
			{	
				double diff = calculateDifference(combinedFrame, lastFrame);
				cout << diff << endl;
				if(diff < DIFF_THRESH)
				{
					stableFrames[numUnchanged % MIN_UNCHANGED] = combinedFrame;
					numUnchanged++;
				}
				else
				{
					numUnchanged = 0;
					isProcessed = false;
					cout << "Change detected: " << diff << endl;
				}
			}

			// needProcess
			if (numUnchanged >= MIN_UNCHANGED && isProcessed == false)
			{
				cout << "numUnchanged = " << numUnchanged << endl;
				cv::Mat averageFrame = cv::Mat::zeros(combinedFrame.size(), combinedFrame.type());
				for (auto frame : stableFrames)
				{
					averageFrame += frame / stableFrames.size();
				}
				show("average", averageFrame);
				if (!lastStableFrame.empty())
				{
					double diff = calculateDifference(lastStableFrame,averageFrame);
					if ( diff > DIFF_THRESH / 2)
					{
						Room nowLighting = getSimilarRoom(averageFrame, vecRP);
						if (lastLighting != nowLighting)
						{
							cout<< getString(nowLighting) <<endl;
							lastLighting = nowLighting;
							sound_controller.setRoom(nowLighting);
#ifdef LIGHT
							setLight(curl, nowLighting);
#endif
						}
						else
						{
							cout << "Equal to lastLighting" << endl;
#ifdef LIGHT
							setLight(curl, lastLighting);
#endif
						}

						isProcessed = true;
						lastStableFrame = averageFrame.clone();
					}
				}
				else
				{
					lastStableFrame = averageFrame.clone();
					Room nowLighting = getSimilarRoom(averageFrame, vecRP);
					lastLighting = nowLighting;
					cout<< getString(nowLighting) <<endl;
					sound_controller.setRoom(nowLighting);
#ifdef LIGHT
					setLight(curl, nowLighting);
#endif
					isProcessed = true;
				}	
			}
		}

		lastFrame = combinedFrame.clone();
 		prevFrame = NowFrame.clone();

        if(cv::waitKey(100) == 27) break;
    }
    cout << endl;
	cout << "Finished grabbing images" << endl; 

    multicam.stop();
	system("pause");
	return 0;
}
