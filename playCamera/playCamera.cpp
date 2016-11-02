// playCamera.cpp : Defines the entry point for the console application.
//

/*
TODO:
Distinguish table and chair based on size
*/

#define SAVE
#define LIGHT
#define TIMER
//#define DEBUG
#define DETECT

#define START_TIME 19
#define END_TIME 22
#define OFF_TIME 7
#define MIN_UNCHANGED 10
#define DIFF_THRESH 1.5
#define MAX_FAILURES 1000

#include "stdafx.h"
#include "helper.h"
#include "CmFile.h"
#include "SoundController.h"
#include "VideoRecorder.h"
#include "MultithreadCam.h"
#include "RectFinder.h"

int _tmain(int argc, _TCHAR* argv[])
{
	MultithreadCam multicam;
	if (!multicam.isInit())
	{
		cout << "Camera init failed! Rebooting in 10 secs..." << endl;
		Sleep(10000);
		rebootSystem();
	}
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

	vector<cv::Mat> stableFrames(MIN_UNCHANGED);
	cv::Mat prevFrame;

	initializeColors();
    RectFinder chair_finder(CHAIR_COLORS, cv::Scalar(0, 0, 255));
    RectFinder table_finder(TABLE_COLORS, cv::Scalar(255, 0, 0));

	int num_failures = 0;

#ifdef MANUAL
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
#endif

#ifdef DEBUG
	Room test_room = STUDY;
	setLight(curl, test_room);
	RectFinder test_finder(TABLE_COLORS, cv::Scalar(0, 255, 0));
	cv::Mat prev;
	while(true)
	{
		cv::Mat f = multicam.getImage();
		cv::Scalar mean = cv::mean(f,  getROIMask());
		test_finder.process(f.clone(), test_room);
		cv::Mat curr = test_finder.drawColorRects();
		show("rects", test_finder.drawColorRects());
		if (!prev.empty()) cout << "diff = " << calculateDifference(prev, curr) << endl;
		prev = curr.clone();
		cv::waitKey();
	}
#endif

	while (true)
    {
		int hour = getCurrentHour();
		int minute = getCurrentMinute();

		if (hour == 11 && minute == 59)
		{
			Sleep(60 * 1000);
			rebootSystem();
		}

#ifdef TIMER
		if (hour < START_TIME || hour >= END_TIME)
		{
#ifdef LIGHT
			if (hour >= OFF_TIME && hour < START_TIME) setLight(curl, OFF);
			else if (hour >= END_TIME || hour < OFF_TIME) setLight(curl, ON);
#endif
			cout << hour << ":" << minute << endl;
			sound_controller.setRoom(OFF);
			cout << "Disable detection!" << endl;
			Sleep(1000 * 60); //wait 1 min
			continue;
		}
		else
		{
			if (lastLighting == OFF)
			{
				setLight(curl, ON);
				lastLighting = ON;
			}
#ifndef DETECT
			Sleep(1000 * 60);
			continue;
#endif
		}
#endif
		
		try
		{
			NowFrame = multicam.getImage();
			if (NowFrame.empty())
			{
				cout << "Failed to gete frame!" << endl;
				continue;
			}
			table_finder.process(NowFrame, lastLighting);
			chair_finder.process(NowFrame, lastLighting);
			cv::Mat combinedFrame = chair_finder.drawColorRects();
			combinedFrame = table_finder.drawColorRects(combinedFrame);

			show("combined", combinedFrame);

			cv::Mat grayFrame;
			cv::cvtColor(combinedFrame, grayFrame, CV_BGR2GRAY);
			//TODO: Check if this is necessary
			//if (cv::countNonZero(grayFrame) > 0)
			{
				if (!lastFrame.empty())
				{	
					double diff = calculateDifference(combinedFrame, lastFrame);
					cout << diff << endl;
					if(diff < DIFF_THRESH)
					{
						stableFrames[numUnchanged % MIN_UNCHANGED] = combinedFrame;
						numUnchanged++;
						cout << "Number of unchanged frames = " << numUnchanged << endl;
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
					isProcessed = true;
					cv::Mat averageFrame = cv::Mat::zeros(combinedFrame.size(), combinedFrame.type());
					for (auto frame : stableFrames)
					{
						averageFrame += frame / stableFrames.size();
					}
					//show("average", averageFrame);
					if (!lastStableFrame.empty())
					{
						double diff = calculateDifference(lastStableFrame,averageFrame);
						if ( diff > DIFF_THRESH / 2)
						{
							Room nowLighting = getSimilarRoom(averageFrame, vecRP);
							if (nowLighting == UNDEFINED)
							{
								if  (lastLighting != UNDEFINED)
								{
									nowLighting = lastLighting;
									setLight(curl, nowLighting);
								}
								else
								{
									lastLighting = ON;
									setLight(curl, ON);
								}
							}
							else //if (nowLighting != UNDEFINED)
							{
								cout << getString(nowLighting) << endl;
								if (lastLighting != nowLighting)
								{
									sound_controller.setRoom(nowLighting);
								}
								else
								{
									cout << "Equal to lastLighting" << endl;
								}
								setLight(curl, nowLighting);
								lastLighting = nowLighting;
							}
							lastLighting = nowLighting;
							lastStableFrame = averageFrame.clone();
						}
						else //	if ( diff <= DIFF_THRESH / 2)
						{
							cout << "Similar to last stable frame" << endl;
							setLight(curl, lastLighting);
						}
					}
					else //if (lastStableFrame.empty())
					{
						lastStableFrame = averageFrame.clone();
						Room nowLighting = getSimilarRoom(averageFrame, vecRP);
						cout << getString(nowLighting) << endl;
						sound_controller.setRoom(nowLighting);
#ifdef LIGHT
						setLight(curl, nowLighting);
#endif
						lastLighting = nowLighting;
					}	
				}
				else if (numUnchanged > 50 && isProcessed == true)
				{
					cout << "Refresh..." << endl;
					numUnchanged = 0;
					setLight(curl, lastLighting);
				}
			}

			lastFrame = combinedFrame.clone();
 			prevFrame = NowFrame.clone();
			if(cv::waitKey(100) == 27) break;
			num_failures = 0;
		}
		catch (cv::Exception exception)
		{
			cout << exception.msg << endl;
			lastLighting = ON;
			setLight(curl, lastLighting);
			Sleep(1000);
			num_failures++;
		}
		catch (std::exception exception)
		{
			cout << exception.what() << endl;
			lastLighting = ON;
			setLight(curl, lastLighting);
			Sleep(1000);
			num_failures++;
		}
		catch (...)
		{
			cout << "Unknown exception!" << endl;
			lastLighting = ON;
			setLight(curl, lastLighting);
			Sleep(1000);
			num_failures++;
		}

		if (num_failures >= MAX_FAILURES)
		{
			cout << "Too many failures, rebooting!" << endl;
			rebootSystem();
		}
    }
    cout << endl;
	cout << "Finished grabbing images" << endl; 

    multicam.stop();
	system("pause");
	return 0;
}
