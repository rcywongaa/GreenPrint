// playCamera.cpp : Defines the entry point for the console application.
//

/*
TODO:
Distinguish table and chair based on size
Use adaptiveThreshold
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

double cosAngle(cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
	 
vector<ColorRect> findColor(cv::Mat input, cv::Scalar color1, cv::Scalar color2, cv::Scalar rect_color)
{
	bool debug = true;
	vector<ColorRect> color_rects;
	if (debug) show("input", input);
	cv::medianBlur(input, input, 31);
	if (debug) show("filtered", input);
	cv::Mat gray;
	cv::cvtColor(input, gray, CV_BGR2GRAY);
	show("gray", gray);
	cv::Mat hsv;
	cv::Mat hsv_array[3];
	cv::Mat mask;
	cv::cvtColor(input, hsv, CV_BGR2HSV);
	cv::split(hsv, hsv_array);
	if (debug) show("h", hsv_array[0]);
	if (debug) show("s", hsv_array[1]);
	if (debug)show("v", hsv_array[2]);
	cv::inRange(hsv, color1, color2, mask);
	cv::bitwise_and(mask, getROIMask(), mask);
	if (debug) show("mask", mask);
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 10);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 10);
	//cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 10);
	//cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	if (debug) show("dilated_mask", mask);

	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> contour_hierarchy;
	cv::findContours(mask, contours, contour_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::Mat contourFrame = cv::Mat::zeros(input.size(), input.type());
	cv::drawContours(contourFrame, contours, -1, cv::Scalar(255, 255, 255));
	cv::Mat rectFrame = cv::Mat::zeros(input.size(), input.type());
	cv::Mat polyFrame = cv::Mat::zeros(input.size(), input.type());
	for (auto contour : contours)
	{
		vector<cv::Point> polygon;
		cv::approxPolyDP(contour, polygon, 0.05 * cv::arcLength(cv::Mat(contour), true), true);
		double maxCos = 0.0;
		vector<float> cos;
		float medianCos = 0.0;
		for (int i = 0; i < polygon.size(); i++)
		{
			int size = polygon.size();
			cos.push_back(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size]));
			//maxCos = std::max(maxCos, abs(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size])));
		}
		int median_idx = cos.size() / 2;
		std::nth_element(cos.begin(), cos.begin() + median_idx, cos.end());
		medianCos = cos[median_idx];
		int npt[] = {polygon.size()}; //Because fillPoly assumes an array polygons...
		const cv::Point* ppt[1] = {&polygon[0]};
		cv::fillPoly(polyFrame, ppt, npt, 1, cv::Scalar(255, 255, 255));

		cv::RotatedRect rect;
		rect = cv::minAreaRect(polygon);

		if (rect.size.height > 100  && rect.size.height < 400 && rect.size.width > 100 && rect.size.width < 400)
		{
			//Consider using average cosAngles
			if (polygon.size() >= 4 && polygon.size() <= 6)
			{
				//if (cv::contourArea(polygon) / rect.size.area() > 0.9)
				if (cv::pointPolygonTest(getROIPts2f(), rect.center, false) > 0)
				{
					ColorRect color_rect;
					color_rect.rect = rect;
					color_rect.color = rect_color;
					color_rects.push_back(color_rect);
					cv::Point2f pts2f[4];
					rect.points(pts2f);
					cv::Point pts[4];
					for (int i = 0; i < 4; i++) pts[i] = pts2f[i];
					cv::fillConvexPoly(rectFrame, &pts[0], 4, cv::Scalar(255, 255, 255));

				}
			}
		}
	}
	if (debug) show("polygon", polyFrame);
	if (debug) show("rect", rectFrame);
	getROIMask();
	return color_rects;
}

vector<ColorRect> findColor(cv::Mat input, tuple<cv::Scalar, cv::Scalar> colors, cv::Scalar rect_color)
{
	return findColor(input, get<0>(colors), get<1>(colors), rect_color);
}

cv::Mat drawColorRects(vector<ColorRect> color_rects, cv::Size size)
{
	cv::Mat rectFrame = cv::Mat(size, CV_8UC3);
	for (auto color_rect : color_rects)
	{
		cv::Mat this_rect_frame(size, CV_8UC3);
		cv::Point2f pts2f[4];
		color_rect.rect.points(pts2f);
		cv::Point pts[4];
		for (int i = 0; i < 4; i++) pts[i] = pts2f[i];
		cv::fillConvexPoly(this_rect_frame, &pts[0], 4, color_rect.color);
		cv::max(this_rect_frame, rectFrame, rectFrame);
	}
	return rectFrame;
}


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

	Room lastLighting = UNDEFINED;
	//setLight(curl, lastLighting);

    // Retrieve an image
	cv::Mat NowFrame;
	cv::Mat background;
	
	cv::Mat bg_mask;
	cv::Ptr<cv::BackgroundSubtractor> p_mog2 = cv::createBackgroundSubtractorMOG2();	

	const int MIN_UNCHANGED = 15;
	const float DIFF_THRESH = 3.0;
	vector<cv::Mat> stableFrames(MIN_UNCHANGED);
	cv::Mat prev_rect_mask;
	cv::Mat prevFrame;
	vector<ColorRect> prev_rects;

	//TODO: Set this parameter
	const float MAX_MOVE = 100;
	vector<cv::Point2f> ROI_PTS2f = getROIPts2f();



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
	while(true)
	{
		cv::Mat f = multicam.getImage();
		cv::cvtColor(f, f, CV_RGB2BGR);
		cv::Scalar mean = cv::mean(f,  getROIMask());
		cv::Mat rectFrame = drawColorRects(findColor(f, TABLE_COLORS[DINING], cv::Scalar(255, 255, 255)), f.size());
		cv::waitKey(100);
	}
	*/

	while (true)
    {
		time_t curr_time = time(NULL);
		tm* curr_time_tm = localtime(&curr_time);
		int hour = curr_time_tm->tm_hour;
		cout << hour << endl;
#ifdef TIMER
		if (hour < 18 || hour >= 22)
		{
#ifdef LIGHT
			if (hour >= 7 && hour < 18) setLight(curl, OFF);
			else if (hour >= 22 || hour < 7) setLight(curl, ON);
#endif
			sound_controller.setRoom(UNDEFINED);
			cout << "Not time yet!" << endl;
			Sleep(1000 * 60);
			continue;
		}
#endif

		cv::Mat roiFrame = NowFrame.clone();

		vector<ColorRect> chair_rects;
		vector<ColorRect> table_rects;
		if (hour > 18 && hour < 19)
		{
			chair_rects = findColor(NowFrame, CHAIR_COLOR1_1800, CHAIR_COLOR2_1800, cv::Scalar(0, 0, 255));// = findTexture(NowFrame, textures); 
			table_rects = findColor(NowFrame, TABLE_COLOR1_1800, TABLE_COLOR2_1800, cv::Scalar(255, 0, 0));// = findRect(NowFrame, background);
		}
		else
		{
			chair_rects = findColor(NowFrame, CHAIR_COLORS[lastLighting], cv::Scalar(0, 0, 255));// = findTexture(NowFrame, textures); 
			table_rects = findColor(NowFrame, TABLE_COLORS[lastLighting], cv::Scalar(255, 0, 0));// = findRect(NowFrame, background);
		}
		//cout << chair_rects.size() << endl;
		cv::Mat chairFrame = drawColorRects(chair_rects, NowFrame.size());
		show("chair", chairFrame);
		cv::Mat tableFrame = drawColorRects(table_rects, NowFrame.size());
		show("table", tableFrame);
		vector<ColorRect> now_rects = chair_rects;
		now_rects.insert(now_rects.end(), table_rects.begin(), table_rects.end());

		vector<ColorRect> true_rects;
		if (now_rects.size() < prev_rects.size())
		{
			for (auto prev_rect = prev_rects.begin(); prev_rect != prev_rects.end(); prev_rect++)
			{
				bool foundMatch = false;
				//TODO: Use min_element instead
				for (auto now_rect = now_rects.begin(); now_rect != now_rects.end(); now_rect++)
				{
					//Check prev_rect->rect.center is inside now_rect->rect
					if (cv::norm(prev_rect->rect.center - now_rect->rect.center) < MAX_MOVE)
					{
						foundMatch = true;
						if (cv::pointPolygonTest(ROI_PTS2f, now_rect->rect.center, true) > 10)
						{	
							true_rects.push_back(*now_rect);
							now_rect = now_rects.erase(now_rect); //Remove rect from future matching and correct now_rect iterator
						}
						break;
					}
					vector<cv::Point2f> now_rect_pts(4);
					now_rect->rect.points(&now_rect_pts[0]);
					if (cv::pointPolygonTest(now_rect_pts, prev_rect->rect.center, false) > 0)
					{
						foundMatch = true;
						true_rects.push_back(*now_rect);
					}
				}
				if (!foundMatch)
				{
					true_rects.push_back(*prev_rect);
				}
			}
		}
		else
		{
			true_rects = now_rects;
		}
		prev_rects = true_rects;

		cv::Mat combinedFrame = drawColorRects(true_rects, NowFrame.size());
		//cv::circle(combinedFrame, cv::Point(ROI_PTS2f[0]), 10, cv::Scalar(255, 255, 255), 10);
		vector<cv::Point> ROI_PTS;
		for (auto roi_pts2f : ROI_PTS2f)
			ROI_PTS.push_back(cv::Point(roi_pts2f));
		//cv::fillConvexPoly(combinedFrame, ROI_PTS, cv::Scalar(255, 255, 255));
		show("combined", combinedFrame);
		//imwrite("example.jpg", combinedFrame);
		//cv::waitKey();

		cv::Mat grayFrame;
		cv::cvtColor(combinedFrame, grayFrame, CV_BGR2GRAY);
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
		cv::cvtColor(lastFrame.clone(), prev_rect_mask, CV_BGR2GRAY, 1);
 		prevFrame = NowFrame.clone();

        if(cv::waitKey(100) == 27) break;
    }
    cout << endl;
	cout << "Finished grabbing images" << endl; 

    multicam.stop();
	system("pause");
	return 0;
}