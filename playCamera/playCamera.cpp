// playCamera.cpp : Defines the entry point for the console application.
//

/*
TODO:
Distinguish table and chair based on size
Use adaptiveThreshold
*/

#define COLOR
#define SAVE
#define LIGHT
//#define TIMER

#include "stdafx.h"
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
	 
vector<ColorRect> findTexture(cv::Mat input, vector<Texture> textures)
{
	cv::Mat texFrame = cv::Mat::zeros(input.size(), CV_8UC3);
	int num_types = textures.size();
	vector<cv::KeyPoint> keypoints;
	vector<tuple<vector<cv::KeyPoint>, vector<cv::KeyPoint>>> matched_keypoints(num_types);
	cv::Mat descriptors;
	//cv::Ptr<cv::AKAZE> p_orb = cv::AKAZE::create();
	cv::Ptr<cv::ORB> p_orb = cv::ORB::create();
	cv::Ptr<cv::DescriptorMatcher> p_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	p_orb->detectAndCompute(input, cv::Mat(), keypoints, descriptors);
	int min_dist = 1e5;
	vector<ColorRect> color_rects;
	for (int type = 0; type < num_types; type++)
	{
		vector<vector<cv::DMatch>> this_matches;
		vector<cv::DMatch> good_matches;
		p_matcher->knnMatch(descriptors, textures[type].descriptors, this_matches, 2);
		float total_dist = 0;
		for (auto best_matches : this_matches)
		{
			if (best_matches[0].distance < 0.8 * best_matches[1].distance) //ratio test
			{
				int idx = get<0>(matched_keypoints[type]).size();
				good_matches.push_back(cv::DMatch(idx, idx, best_matches[0].distance));
				get<0>(matched_keypoints[type]).push_back(keypoints[best_matches[0].queryIdx]);
				get<1>(matched_keypoints[type]).push_back(textures[type].keypoints[best_matches[0].trainIdx]);
				total_dist += best_matches[0].distance;
			}
		}
		if (get<0>(matched_keypoints[type]).size() < 4)
		{
			//cout << "Not enough matches: " << type << endl;
			continue;
		}
		cv::Mat matchFrame;
		cv::drawMatches(input, get<0>(matched_keypoints[type]), textures[type].image, get<1>(matched_keypoints[type]), good_matches, matchFrame);
		//show("match - " + to_string(type), matchFrame);
		//TODO: partition?
		vector<tuple<vector<cv::Point2f>, vector<cv::Point2f>>> clusters(textures[type].max_instances);
		vector<int> labels;
		vector<cv::Point2f> cluster_pts;
		for (auto keypoint : get<0>(matched_keypoints[type]))
		{
			cluster_pts.push_back(keypoint.pt);
		}
		double compactness = cv::kmeans(cluster_pts, textures[type].max_instances, labels, cv::TermCriteria(cv::TermCriteria::EPS, -1, 1.0), 3, cv::KMEANS_PP_CENTERS);
		for (int i = 0; i < labels.size(); i++)
		{
			get<0>(clusters[labels[i]]).push_back(get<0>(matched_keypoints[type])[i].pt);
			get<1>(clusters[labels[i]]).push_back(get<1>(matched_keypoints[type])[i].pt);
		}

		cv::Mat clusterFrame = input.clone();
		cv::Mat homoFrame = input.clone();
		for (auto cluster : clusters)
		{
			cv::Scalar cluster_color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
			for (int i = 0; i < get<0>(cluster).size(); i++)
			{
				circle(clusterFrame, get<0>(cluster)[i], 20, cluster_color, 10);
			}
			cv::Mat homography;
			homography = cv::findHomography(get<1>(cluster), get<0>(cluster), CV_RANSAC);
			vector<cv::Point2f> object_bb;					 
			int width = textures[type].image.cols;
			int height = textures[type].image.rows;
			vector<cv::Point2f> tex_bb;
			tex_bb.push_back(cv::Point2f(0, 0));
			tex_bb.push_back(cv::Point2f(width, 0));
			tex_bb.push_back(cv::Point2f(width, height));
			tex_bb.push_back(cv::Point2f(0, height));
			if (!homography.empty())
			{
				cv::perspectiveTransform(tex_bb, object_bb, homography);
				//cout << homography << endl;
				//cout << object_bb << endl;
				assert(object_bb.size() == 4);
				cv::Point pts[4];
				for (int i = 0; i < 4; i++)
				{
					pts[i] = object_bb[i];
					circle(homoFrame, pts[i], 20, cv::Scalar(0, 0, 255), 10);
				}
				double maxCos = 0.0;
				for (int i = 0; i < 4; i++)
				{
					maxCos = std::max(maxCos, abs(cosAngle(pts[i], pts[(i+2) % 4], pts[(i+1) % 4])));
				}
				if (maxCos < 0.5)
				{
					cv::fillConvexPoly(homoFrame, pts, 4, textures[type].color);
					cv::fillConvexPoly(texFrame, pts, 4, textures[type].color);
					ColorRect color_rect;
					color_rect.rect = cv::RotatedRect(cv::Point2f(pts[0]), cv::Point2f(pts[1]), cv::Point2f(pts[2]));
					color_rect.color = textures[type].color;
					color_rects.push_back(color_rect);
				}
				// else cout << "Bad homography matrix" << endl;
			}
			//else cout << "Homography failed" << endl;
		}
		//show("cluster - " + to_string(type), clusterFrame);
		//show("homo - " + to_string(type), homoFrame);
	}
	show("tex", texFrame);
	return color_rects;
}

vector<ColorRect> findRect(cv::Mat input, cv::Mat background)
{
	if (background.size() != input.size())
	{
		cout << "Warning: Auto-resizing background to " << input.size() << endl;
		cv::resize(background, background, input.size());
	}
	show("input", input);
	show("background", background);
	cv::Mat filteredFrame;

	//TODO: Perhaps use absdiff

	//cv::subtract(input, background, filteredFrame);
	//filteredFrame = cv::abs(filteredFrame);
	cv::absdiff(input, background, filteredFrame);
	show("subtracted", filteredFrame);

	//cv::threshold(filteredFrame, filteredFrame, 100, 255, CV_THRESH_BINARY); //Emphasize all edges
	cv::medianBlur(filteredFrame, filteredFrame, 11);
	cv::adaptiveThreshold(filteredFrame, filteredFrame, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 13, -1);
	//TODO: Consider erode + dilate
	show("filtered", filteredFrame);

	cv::Mat edgeFrame;
	cv::Canny(filteredFrame, edgeFrame, 0, 300, 3, true);
	int DILATE_SIZE = 3;
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DILATE_SIZE, DILATE_SIZE), cv::Point(DILATE_SIZE / 2, DILATE_SIZE / 2));
	cv::dilate(edgeFrame, edgeFrame, element); // Fill holes in contours
	//TODO: Do line thinning here?
	show("edges", edgeFrame);

	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> contour_hierarchy;
	//cv::findContours(edgeFrame, contours, contour_hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	cv::findContours(edgeFrame, contours, contour_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::Mat contourFrame = cv::Mat::zeros(input.size(), input.type());
	cv::drawContours(contourFrame, contours, -1, cv::Scalar(255, 255, 255));
	show("contours", contourFrame);

	cv::Mat rectFrame = cv::Mat::zeros(input.size(), input.type());
	cv::Mat polyFrame = cv::Mat::zeros(input.size(), input.type());
	vector<ColorRect> color_rects;
	for (auto contour : contours)
	{
		vector<cv::Point> polygon;
		cv::approxPolyDP(contour, polygon, 0.05 * cv::arcLength(cv::Mat(contour), true), true);
		//cv::convexHull(contour, polygon);
		double maxCos = 0.0;
		for (int i = 0; i < polygon.size(); i++)
		{
			int size = polygon.size();
			maxCos = std::max(maxCos, abs(cosAngle(polygon[i], polygon[(i+2) % size], polygon[(i+1) % size])));
		}
		int npt[] = {polygon.size()}; //Because fillPoly assumes an array polygons...
		const cv::Point* ppt[1] = {&polygon[0]};
		cv::fillPoly(polyFrame, ppt, npt, 1, cv::Scalar(255, 255, 255));

		cv::RotatedRect rect;
		rect = cv::minAreaRect(polygon);

		if (rect.size.height > 25 && rect.size.width > 25)
		{
			//Consider using average cosAngles
			if (polygon.size() == 4 && maxCos < 0.5)
			{
				//TODO: The following parameters should be fine-tuned
				// Perhaps sort rect sizes and find outliers
				//if (rect.size.height > 50 && rect.size.width > 50 && cv::contourArea(polygon) / rect.size.area() > 0.9)
				if (true)
				{
					cv::Point2f pts2f[4];
					rect.points(pts2f);
					cv::Point pts[4];
					for (int i = 0; i < 4; i++) pts[i] = pts2f[i];
					cv::fillConvexPoly(rectFrame, &pts[0], 4, cv::Scalar(255, 255, 255));
					ColorRect color_rect;
					color_rect.rect = rect;
					color_rect.color = cv::Scalar(255, 255, 255);
					color_rects.push_back(color_rect);
				}
			}
		}
	}
	show("polygon", polyFrame);
	show("rect", rectFrame);
	//cv::waitKey();
	return color_rects;
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

vector<cv::Mat> generateTemplates(cv::Mat input)
{
	float PAD_SCALE = 1.5;
	vector<cv::Mat> pyramid;
	cv::Mat cropped;
	cv::Point2f centroid;
	float radius;
	 cv::Mat edgeFrame;
	cv::Canny(input, edgeFrame, 0, 100, 3, true);
	vector<vector<cv::Point>> contours;
	cv::findContours(edgeFrame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<cv::Point> all_contours;
	for (auto contour : contours)
		all_contours.insert(all_contours.end(), contour.begin(), contour.end());
	cv::minEnclosingCircle(all_contours, centroid, radius);
	cv::Mat drawFrame = cv::Mat::zeros(input.size(), input.type());
	cv::drawContours(drawFrame, contours, -1, cv::Scalar(255, 255, 255));
	cv::circle(drawFrame, centroid, radius, cv::Scalar(255, 0, 0));
	//show("draw", drawFrame);
	//show("tmp", input);
	//cv::waitKey(1);
	cropped = input(cv::Rect(centroid.x - radius, centroid.y - radius, 2*radius, 2*radius)).clone();

	for (float i = 1; i > 0.99; i -= 0.1)
	{
		cv::Size size(cropped.size().width * i, cropped.size().height * i); 
		cv::Mat scaled;
		cv::resize(cropped, scaled, size);
		for (double angle = 0; angle < 359.9; angle += 90)
		{
			cv::Mat scaled_rotated;
			cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2f(scaled.size().width / 2, scaled.size().height / 2), angle, 1);
			cv::warpAffine(scaled, scaled_rotated, rot_mat, scaled.size());
			cv::Size pad_size;
			if (scaled_rotated.cols * PAD_SCALE > cropped.cols || scaled_rotated.rows * PAD_SCALE > cropped.rows)
				pad_size = cropped.size();
			else
				pad_size = cv::Size(scaled_rotated.cols * PAD_SCALE, scaled_rotated.rows * PAD_SCALE);
			cv::Mat padded = cv::Mat::ones(pad_size, scaled_rotated.type()) * 0;
			scaled_rotated.copyTo(padded(cv::Rect(cv::Point((padded.cols - scaled_rotated.cols) / 2, (padded.rows - scaled_rotated.rows) / 2), cv::Size(scaled_rotated.cols, scaled_rotated.rows))));
			pyramid.push_back(padded.clone());
			//pyramid.push_back(scaled_rotated.clone());
		}
	}
	return pyramid;
}

bool initCurl(CURL* &curl)
{
	/********** CURL LIGHTING CONTROL SETUP **********/
	CURLcode curl_res;
	curl_global_init(CURL_GLOBAL_DEFAULT);
	curl = curl_easy_init();
	int CLUSTER_IDX = 3; //only 1 - 4
	unsigned char INITIAL_COLOR[3] = {255, 255, 255}; 
	if (!curl)
	{
		cout << "Unable to connect to lighting... exiting" << endl;
		return false;
	}	
	/*
	curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.211/led_control.php?name=GP" + to_string(CLUSTER_IDX) + 
		"&rgb_value=" + to_string(INITIAL_COLOR[0]) + "," + to_string(INITIAL_COLOR[1]) + "," + to_string(INITIAL_COLOR[2]));
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // Follow redirection
	curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L); // Skip SSL verification
	curl_res = curl_easy_perform(curl);
	if (curl_res != CURLE_OK)
	{
		cout << "Setting initial lighting failed!" << endl;
		curl_easy_cleanup(curl);
		return false;
	}
	*/
	return true;
}

void sendUrl(CURL* curl, int gp, int r, int g, int b)
{
	CURLcode curl_res;
	curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.211/led_control.php?name=GP" + to_string(gp) + 
		"&rgb_value=" + to_string(r) + "," + to_string(g) + "," + to_string(b));
	curl_res = curl_easy_perform(curl);
	if (curl_res != CURLE_OK)
	{
		cout << "Unable to set light..." << endl;
	}
}

void setLight(CURL* curl, Room room)
{
	CURLcode curl_res;
	int DELAY = 1500;
	switch (room)
	{
			
		case DINING:
			cout << "Changing to dining lights..." << endl;
			sendUrl(curl, 3, 255, 60, 60);
			Sleep(DELAY);
			sendUrl(curl, 4, 255, 60, 60);
			Sleep(DELAY);
			sendUrl(curl, 6, 235, 50, 0);
			Sleep(DELAY);
			sendUrl(curl, 5, 255, 60, 60);
			Sleep(DELAY);
			sendUrl(curl, 1, 255, 60, 60);
			Sleep(DELAY);
			sendUrl(curl, 2, 255, 60, 60);
			Sleep(DELAY);
			sendUrl(curl, 8, 255, 40, 40);
			Sleep(DELAY);
			sendUrl(curl, 7, 255, 40, 40);
			Sleep(DELAY);
			break;
		case UNDEFINED:
		case LIVING:
			cout << "Changing to living lights..." << endl;
			sendUrl(curl, 3, 235, 50, 0);
			Sleep(DELAY);
			sendUrl(curl, 4, 235, 50, 0);
			Sleep(DELAY);
			sendUrl(curl, 6, 255, 70, 50);
			Sleep(DELAY);
			sendUrl(curl, 5, 235, 50, 0);
			Sleep(DELAY);
			sendUrl(curl, 1, 235, 50, 0);
			Sleep(DELAY);
			sendUrl(curl, 2, 255, 70, 0);
			Sleep(DELAY);
			sendUrl(curl, 8, 255, 70, 0);
			Sleep(DELAY);
			sendUrl(curl, 7, 255, 70, 0);
			Sleep(DELAY);
			break;
		case STUDY:
			cout << "Changing to study lights..." << endl;
			sendUrl(curl, 3, 50, 50, 100);
			Sleep(DELAY);
			sendUrl(curl, 4, 50, 50, 100);
			Sleep(DELAY);
			sendUrl(curl, 6, 255, 70, 50);
			Sleep(DELAY);
			sendUrl(curl, 5, 50, 50, 100);
			Sleep(DELAY);
			sendUrl(curl, 1, 255, 180, 180);
			Sleep(DELAY);
			sendUrl(curl, 2, 255, 180, 180);
			Sleep(DELAY);
			sendUrl(curl, 8, 255, 180, 180);
			Sleep(DELAY);
			sendUrl(curl, 7, 255, 180, 180);
			Sleep(DELAY);
			break;
		case ON:
			cout << "Switching on..." << endl;
			for (int i = 1; i <= 8; i++)
			{
				sendUrl(curl, i, 255, 255, 255);
				Sleep(DELAY);
			}
			break;
		case OFF:
			cout << "Switching off..." << endl;
			for (int i = 1; i <= 8; i++)
			{
				sendUrl(curl, i, 0, 0, 0);
				Sleep(DELAY);
			}
			break;
		default:
			cout << "Unknown style" << endl;
			return;
	}
}

vector<RoomPhotos> prepareData(){
	//Use full resolution for rectcosAngle extraction, resize results
	vector<RoomPhotos> vecRP;
	vecRP.reserve(100);
	cv::Size TRAIN_IMAGE_SIZE(640, 480);

	string wkdir = "D:/TrainData/";
	string background = wkdir + "background.jpg";
	cv::Mat bg_gray = cv::imread(background, CV_LOAD_IMAGE_GRAYSCALE);

	for (int k = 0; k < 3; k++)
	{
		vecS namesNE;
		string imgName = wkdir + getString(static_cast<Room>(k)) + "/*.png";
		int imgNum = CmFile::GetNamesNE(imgName, namesNE);
		for (int i = 0; i < imgNum; i++)
		{
			RoomPhotos temp;
			cv::Mat train;
			struct stat st;

			if (stat((wkdir + getString(static_cast<Room>(k)) + "/" + namesNE[i] + ".png").c_str(), &st) == 0)
			{
				train = cv::imread(wkdir + getString(static_cast<Room>(k)) + "/" + namesNE[i] + ".png", CV_LOAD_IMAGE_COLOR); 
				cv::resize(train, train, TRAIN_IMAGE_SIZE);
			}
			else
			{
				cout << "Missing training data" << endl;
				/*
				cv::Mat gray = cv::imread(wkdir + getString(static_cast<Room>(k)) + "/" + namesNE[i] + ".jpg", CV_LOAD_IMAGE_GRAYSCALE);
				train = drawColorRects(findRect(gray, bg_gray), gray.size());
				//cv::resize(train, train,imgsize);
				cv::cvtColor(train, train, CV_GRAY2BGR);
				cv::imwrite(wkdir + getString(static_cast<Room>(k)) + "/filtered/" + namesNE[i] + "_filtered.jpg", train);
				*/
			}
			temp.img = CImg<uchar>(train);
			temp.score = 0;
			temp.room = static_cast<Room>(k);
			vecRP.push_back(temp);
		}
	}
	return vecRP;
}

double calcScore(cv::Mat img, cv::Mat train)
{
	// a) resize, center and run phash on different rotations
	// b) encode rectangle positions and orientations and compare
	// c) template matching with multiscale multiorientation training data

	cv::resize(img, img, train.size());
	cv::Mat result;
	double min = 1e5;
	double max = 0.0;
	for (auto tmpl : generateTemplates(train))
	{
		double this_max;
		double this_min;
		cv::matchTemplate(img, tmpl, result, CV_TM_CCOEFF_NORMED);
		cv::Point max_pos;
		cv::Point min_pos;
		cv::minMaxLoc(result, &this_min, &this_max, &min_pos, &max_pos);

		if (this_max > max)
		{
			max = this_max;
		}
	}
	return max;
}

//Use normalized distance and normalized orientation to compare
Room getSimilarRoom(cv::Mat &img, vector<RoomPhotos> &vecRP){
	
	CImg<uchar> img1 = CImg<uchar>(img);

	cout << "Finding match... " << endl;
	for (int i = 0; i < (int)vecRP.size(); i++)
	{
		//_ph_compare_images(img1, vecRP[i].img, vecRP[i].score);
		vecRP[i].score = calcScore(img, vecRP[i].img.get_MAT());
	}

	RoomPhotos maxRP;
	maxRP.score = 0;

	for (auto &rp : vecRP)
	{
		if (rp.score > maxRP.score)	{
			maxRP = rp;
		}
	}

	cout << "Score: " << maxRP.score << endl;
	if (maxRP.score < 0.5)
	{
		return Room::UNDEFINED;
	}else
	{
		show("most_similar", cv::Mat(maxRP.img.get_MAT()));
		return maxRP.room;
	}
}

//TODO: Check values
double calculateDifference(cv::Mat &img1, cv::Mat &img2){
	cv::Mat xor;
	cv::bitwise_xor(img1, img2, xor);
	return cv::mean(xor)[0];
	/*
	CImg<uchar> imgLeft = CImg<uchar>(img1);
	CImg<uchar> imgRight = CImg<uchar>(img2);

	double pcc;
	_ph_compare_images(imgLeft, imgRight, pcc);
	
	return pcc;
	*/
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

#ifndef COLOR
	NowFrame = cv::Mat(rows, cols, CV_8UC1, rawImage.GetData()).clone();

	// If background image exists and less than 1 hour old, use it
	struct stat st;
	if (stat("background.jpg", &st) == 0 && time(NULL) - st.st_mtime < 12*60*60)
	{
		cout << "Using old background" << endl;
		background = cv::imread("background.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	}
	else
	{
		cout << "Capturing new background" << endl;
		background = NowFrame.clone();
		cv::imwrite("background.jpg", background);
	}
	cv::resize(background, background, NowFrame.size());
#endif
	
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

	//6:00pm
	/*
	cv::Scalar TABLE_COLOR1(0, 0, 0);
	cv::Scalar TABLE_COLOR2(40, 40, 60);
	cv::Scalar CHAIR_COLOR1(0, 40, 0);
	cv::Scalar CHAIR_COLOR2(40, 100, 75);
	*/
	//6:30pm
	/*
	cv::Scalar TABLE_COLOR1(0, 0, 0);
	cv::Scalar TABLE_COLOR2(40, 50, 60);
	cv::Scalar CHAIR_COLOR1(0, 50, 0);
	cv::Scalar CHAIR_COLOR2(30, 120, 75);
	*/
	//7:00pm
	/*
	cv::Scalar TABLE_COLOR1(0, 0, 0);
	cv::Scalar TABLE_COLOR2(255, 50, 60);
	cv::Scalar CHAIR_COLOR1(0, 30, 0);
	cv::Scalar CHAIR_COLOR2(30, 120, 75);
	*/												 
	cv::Scalar TABLE_COLOR1_1800(0, 0, 0);
	cv::Scalar TABLE_COLOR2_1800(40, 50, 60);
	cv::Scalar CHAIR_COLOR1_1800(0, 40, 0);
	cv::Scalar CHAIR_COLOR2_1800(40, 120, 75);

	//Living
	cv::Scalar LIVING_TABLE_COLOR1(0, 50, 0);
	cv::Scalar LIVING_TABLE_COLOR2(15, 175, 75);
	cv::Scalar LIVING_CHAIR_COLOR1(0, 150, 50);
	cv::Scalar LIVING_CHAIR_COLOR2(15, 200, 175);

	//Dining
	cv::Scalar DINING_TABLE_COLOR1(0, 50, 0);
	cv::Scalar DINING_TABLE_COLOR2(15, 125, 75);
	cv::Scalar DINING_CHAIR_COLOR1(0, 125, 50);
	cv::Scalar DINING_CHAIR_COLOR2(15, 175, 150);

	//Study
	cv::Scalar STUDY_TABLE_COLOR1(0, 0, 0);
	cv::Scalar STUDY_TABLE_COLOR2(255, 255, 50);
	cv::Scalar STUDY_CHAIR_COLOR1(0, 0, 25);
	cv::Scalar STUDY_CHAIR_COLOR2(30, 75, 100);

	vector<tuple<cv::Scalar, cv::Scalar>> CHAIR_COLORS(NUM_ROOMS);
	vector<tuple<cv::Scalar, cv::Scalar>> TABLE_COLORS(NUM_ROOMS);
	CHAIR_COLORS[LIVING] = tuple<cv::Scalar, cv::Scalar>(LIVING_CHAIR_COLOR1, LIVING_CHAIR_COLOR2);
	TABLE_COLORS[LIVING] = tuple<cv::Scalar, cv::Scalar>(LIVING_TABLE_COLOR1, LIVING_TABLE_COLOR2);
	CHAIR_COLORS[UNDEFINED] = tuple<cv::Scalar, cv::Scalar>(LIVING_CHAIR_COLOR1, LIVING_CHAIR_COLOR2);
	TABLE_COLORS[UNDEFINED] = tuple<cv::Scalar, cv::Scalar>(LIVING_TABLE_COLOR1, LIVING_TABLE_COLOR2);
	CHAIR_COLORS[DINING] = tuple<cv::Scalar, cv::Scalar>(DINING_CHAIR_COLOR1, DINING_CHAIR_COLOR2);
	TABLE_COLORS[DINING] = tuple<cv::Scalar, cv::Scalar>(DINING_TABLE_COLOR1, DINING_TABLE_COLOR2);
	CHAIR_COLORS[STUDY] = tuple<cv::Scalar, cv::Scalar>(STUDY_CHAIR_COLOR1, STUDY_CHAIR_COLOR2);
	TABLE_COLORS[STUDY] = tuple<cv::Scalar, cv::Scalar>(STUDY_TABLE_COLOR1, STUDY_TABLE_COLOR2);

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
#ifdef COLOR
		//NowFrame = multicam.getImage();
		cv::cvtColor(NowFrame, NowFrame, CV_RGB2BGR);
#else
		NowFrame = cv::Mat(rows, cols, CV_8UC1, rawImage.GetData()).clone();
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