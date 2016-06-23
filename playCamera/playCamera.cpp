// playCamera.cpp : Defines the entry point for the console application.
//

/*
TODO:
Test update background image / Capture backgrounds under different lights
Test handle occlusion
Zoom / ROI from camera image
ROI for tracking
New training layouts
Test delete old records
*/

#define COLOR
//#define SAVE

#include "stdafx.h"
#include "CmFile.h"
#include "SoundController.h"

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    
	ostringstream version;
	version << "FlyCapture2 library version: " << fc2Version.major << "." << fc2Version.minor << "." << fc2Version.type << "." << fc2Version.build;
	cout << version.str() <<endl;  
    
	ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
	cout << timeStamp.str() << endl << endl;  
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    cout << endl;
	cout << "*** CAMERA INFORMATION ***" << endl;
	cout << "Serial number -" << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;
	
}

void PrintFormat7Capabilities( Format7Info fmt7Info )
{
    cout << "Max image pixels: (" << fmt7Info.maxWidth << ", " << fmt7Info.maxHeight << ")" << endl;
	cout << "Image Unit size: (" << fmt7Info.imageHStepSize << ", " << fmt7Info.imageVStepSize << ")" << endl;
    cout << "Offset Unit size: (" << fmt7Info.offsetHStepSize << ", " << fmt7Info.offsetVStepSize << ")" << endl;
	cout << "Pixel format bitfield: 0x" << fmt7Info.pixelFormatBitField << endl;
	
}

void show(string window, cv::Mat img)
{
	cv::Mat resized;
	cv::resize(img, resized, cv::Size(320, 240));
	//cv::resize(img, resized, cv::Size(640, 480));
	imshow(window, resized);
}

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
	bool debug = false;
	vector<ColorRect> color_rects;
	if (debug) show("input", input);
	cv::medianBlur(input, input, 15);
	if (debug) show("filtered", input);
	cv::Mat hsv;
	cv::Mat hsv_array[3];
	cv::Mat mask;
	cv::cvtColor(input, hsv, CV_BGR2HSV);
	cv::split(hsv, hsv_array);
	if (debug) show("h", hsv_array[0]);
	if (debug) show("s", hsv_array[1]);
	if (debug)show("v", hsv_array[2]);
	cv::inRange(hsv, color1, color2, mask);
	if (debug) show("mask", mask);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 15);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 10);
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
	return color_rects;
}

cv::Mat drawColorRects(vector<ColorRect> color_rects, cv::Size size)
{
	cv::Mat rectFrame = cv::Mat(size, CV_8UC3);
	for (auto color_rect : color_rects)
	{
		cv::Point2f pts2f[4];
		color_rect.rect.points(pts2f);
		cv::Point pts[4];
		for (int i = 0; i < 4; i++) pts[i] = pts2f[i];
		cv::fillConvexPoly(rectFrame, &pts[0], 4, color_rect.color);
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
	cropped = input(cv::Rect(centroid.x - radius, centroid.y - radius, 2*radius, 2*radius)).clone();

	for (float i = 1; i > 0.201; i -= 0.1)
	{
		cv::Size size(cropped.size().width * i, cropped.size().height * i); 
		cv::Mat scaled;
		cv::resize(cropped, scaled, size);
		for (double cosAngle = 0; cosAngle < 359.9; cosAngle += 15)
		{
			cv::Mat scaled_rotated;
			cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2f(scaled.size().width / 2, scaled.size().height / 2), cosAngle, 1);
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

bool getAudio(IAudioEndpointVolume* &endpointVolume)
{
	CoInitialize(NULL);
	IMMDeviceEnumerator *deviceEnumerator = NULL;
	HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL, CLSCTX_INPROC_SERVER, __uuidof(IMMDeviceEnumerator), (LPVOID *)&deviceEnumerator);
	IMMDevice *defaultDevice = NULL;

	hr = deviceEnumerator->GetDefaultAudioEndpoint(eRender, eConsole, &defaultDevice);
	deviceEnumerator->Release();
	deviceEnumerator = NULL;

	hr = defaultDevice->Activate(__uuidof(IAudioEndpointVolume), CLSCTX_INPROC_SERVER, NULL, (LPVOID *)&endpointVolume);
	defaultDevice->Release();
	defaultDevice = NULL;
	return true;
}

void setSound(IAudioEndpointVolume *audio, Room room)
{
	for (int i = 100; i >= 0; i--)
	{
		audio->SetMasterVolumeLevelScalar(i/100.0, NULL);
		Sleep(50);
	}
	
	switch (room)
	{
		case LIVING:
			PlaySound(L"D:/sound/canon_in_d.wav", NULL, SND_LOOP | SND_ASYNC);
			break;
		case DINING:
			PlaySound(L"D:/sound/one_summers_day.wav", NULL, SND_LOOP | SND_ASYNC);
			break;
		case STUDY:
			PlaySound(L"D:/sound/ocean.wav", NULL, SND_LOOP | SND_ASYNC);
			break;
		default:
			PlaySound(NULL, NULL, SND_LOOP | SND_ASYNC); // to stop music
			break;
	}

	for (int i = 0; i <= 100; i++)
	{
		audio->SetMasterVolumeLevelScalar(i/100.0, NULL);
		Sleep(50);
	}
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
	return true;
}

void setLight(CURL* curl, Room room)
{
	int CLUSTER_IDX = 3;
	unsigned char light_color[3] = {0, 0, 0};
	switch (room)
	{
		case UNDEFINED:
			cout << "Switching off..." << endl;
			light_color[0] = 0;
			light_color[1] = 0;
			light_color[2] = 0;
			break;
		case DINING:
			cout << "Changing to dining lights..." << endl;
			light_color[0] = 255;
			light_color[1] = 128;
			light_color[2] = 128;
			break;
		case LIVING:
			cout << "Changing to living lights..." << endl;
			light_color[0] = 128;
			light_color[1] = 255;
			light_color[2] = 128;
			break;
		case STUDY:
			cout << "Changing to study lights..." << endl;
			light_color[0] = 128;
			light_color[1] = 128;
			light_color[2] = 255;
			break;
		default:
			cout << "Unknown style" << endl;
			return;
	}

	curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.211/led_control.php?name=GP" + to_string(CLUSTER_IDX) + 
		"&rgb_value=" + to_string(light_color[0]) + "," + to_string(light_color[1]) + "," + to_string(light_color[2]));
	CURLcode curl_res;
	curl_res = curl_easy_perform(curl);
	if (curl_res != CURLE_OK)
	{
		cout << "Unable to set light..." << endl;
	}
	else
		Sleep(5000);
}

vector<RoomPhotos> prepareData(cv::Size &imgsize);
Room getSimilarRoom(cv::Mat &img, vector<RoomPhotos> &vecRP);
double calculateDifference(cv::Mat &img1, cv::Mat &img2);

int _tmain(int argc, _TCHAR* argv[])
{
	/*
	const int CHAIR = 0;
	const int TABLE = 1;
	vector<Texture> textures(2);
	vector<string> texture_names;
	//texture_names.push_back("D:/chair_texture.jpg");
	//texture_names.push_back("D:/table_texture.jpg");
	textures[CHAIR].image = cv::imread("D:/chair_texture.jpg");
	textures[TABLE].image = cv::imread("D:/table_texture.jpg");
	//cv::Ptr<cv::AKAZE> p_orb = cv::AKAZE::create();
	cv::Ptr<cv::ORB> p_orb = cv::ORB::create();
	for (int type = 0; type < 2; type++)
	{
		p_orb->detectAndCompute(textures[type].image, cv::noArray(), textures[type].keypoints, textures[type].descriptors);

		cv::Mat keypointFrame;
		cv::drawKeypoints(textures[type].image, textures[type].keypoints, keypointFrame);
		//show("keypoints - " + to_string(type), keypointFrame);
		textures[type].max_instances = 4;
		textures[type].color = cv::Scalar(255, 0, 0);
		textures[type].type = type;
	}
	*/

	/********** Test code **********/
	/*
	SoundController sc;
	//sc.setRoom(STUDY);
	//Sleep(10000);
	//sc.setRoom(DINING);
	cv::Mat in;
	in = cv::imread("D:/tex_test.png", CV_LOAD_IMAGE_COLOR);
	cv::putText(in, getCurrentTime(), cv::Point(0, in.rows), CV_FONT_HERSHEY_PLAIN, 10, cv::Scalar(255, 255, 255), 10);
	show("text", in);
	//findColor(in);
	cv::waitKey();
	sc.setRoom(DINING);

	in = cv::imread("D:/shadow_test.jpg");
	show("original", in);
	cv::Mat mask = cv::imread("D:/mask.png", CV_LOAD_IMAGE_GRAYSCALE);
	int DILATE_SIZE = 3;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DILATE_SIZE, DILATE_SIZE), cv::Point(DILATE_SIZE / 2, DILATE_SIZE / 2));
	cv::dilate(mask, mask, element);
	cv::inpaint(in, mask, in, 3, cv::INPAINT_TELEA);
	show("inpaint", in);
	cv::waitKey();
	  */
	/*
	cv::Mat tex_test = cv::imread("D:/tex_test.png");
	findTexture(tex_test, textures);
	cv::waitKey();
	*/
	/*
	vector<RoomPhotos> testData = prepareData(cv::Size(640, 480));
	cv::Mat test = cv::imread("D:/failed1.jpg", CV_LOAD_IMAGE_COLOR);
	cout << getString(getSimilarRoom(test, testData)) << endl;
	cv::waitKey();
	*/

	/********** Set up sound control **********/
	IAudioEndpointVolume *audio = NULL;
	if (!getAudio(audio))
	{
		return -1;
	}
#ifdef CURL
	CURL* curl;
	if (!initCurl(curl))
		return -1;
	setLight(curl, "dining");
	setLight(curl, "living");
	setLight(curl, "study");
	setLight(curl, "off");
#endif
	PrintBuildInfo();
	BusManager busMgr;
    unsigned int numCameras;
	CheckError(busMgr.GetNumOfCameras(&numCameras));
    cout << "Number of cameras detected: " << numCameras <<endl; 

	if ( numCameras < 1 )
    {
        cout << "Insufficient number of cameras... exiting" << endl; 
        system("pause");
		return -1;
    }

    PGRGuid guid;
    CheckError(busMgr.GetCameraFromIndex(0, &guid));

	GigECamera cam;
    // Connect to a camera
	CheckError(cam.Connect(&guid));

	// Get the camera information
    CameraInfo camInfo;
	CheckError(cam.GetCameraInfo(&camInfo));

    PrintCameraInfo(&camInfo);   
 
	GigEImageSettingsInfo imageInfo;
	CheckError(cam.GetGigEImageSettingsInfo(&imageInfo));
	if ((PIXEL_FORMAT_BGR & imageInfo.pixelFormatBitField) == 0)
	{
		cout << "Pixel format is not supported" << endl;
		return -1;
	}
	
	Mode mode = MODE_0;
	CheckError(cam.SetGigEImagingMode(mode));
	  
	unsigned int maxPacketSize;
	CheckError(cam.DiscoverGigEPacketSize(&maxPacketSize));
	cout << "Max packet size = " << maxPacketSize << endl;

	GigEProperty packetSizeProp;
	packetSizeProp.propType = PACKET_SIZE;
	packetSizeProp.isReadable = true;
	packetSizeProp.value = maxPacketSize;
	CheckError(cam.SetGigEProperty(&packetSizeProp));
	cout << "Packet size = " << maxPacketSize << endl;
	int PACKET_DELAY_VAL = 0;
	GigEProperty packetDelayProp;
	packetDelayProp.propType = PACKET_DELAY;
	packetDelayProp.isReadable = true;
	packetDelayProp.value = PACKET_DELAY_VAL;
	CheckError(cam.SetGigEProperty(&packetDelayProp));
	cout << "Packet delay = " << PACKET_DELAY_VAL << endl;

	Property brightnessProp;
	brightnessProp.type = BRIGHTNESS;
	brightnessProp.absValue = 25; //% (0-25)
	brightnessProp.autoManualMode = false;
	brightnessProp.onOff = true;
	brightnessProp.absControl = true;
	CheckError(cam.SetProperty(&brightnessProp));
	CheckError(cam.GetProperty(&brightnessProp));
	cout << "Brightness = " << brightnessProp.absValue << endl;
	Property exposureProp;
	exposureProp.type = AUTO_EXPOSURE;
#ifdef COLOR
	exposureProp.absValue = 1.25;
#else
	exposureProp.absValue = 2.00; //EV (-7.5 - 2.5)
#endif
	exposureProp.autoManualMode = false;
	exposureProp.onOff = true;
	exposureProp.absControl = true;
	CheckError(cam.SetProperty(&exposureProp));
	CheckError(cam.GetProperty(&exposureProp));
	cout << "Exposure = " << exposureProp.absValue << endl;
	Property sharpnessProp;
	sharpnessProp.type = SHARPNESS;
	sharpnessProp.valueA = 1048; //abs (0 - 4095)
	sharpnessProp.autoManualMode = false;
	sharpnessProp.onOff = true;
	sharpnessProp.absControl = false; //absControl not supported
	CheckError(cam.SetProperty(&sharpnessProp));
	CheckError(cam.GetProperty(&sharpnessProp));
	cout << "Sharpness = " << sharpnessProp.valueA << endl;
	Property gammaProp;
	gammaProp.type = GAMMA;
#ifdef COLOR
	gammaProp.absValue = 0.5;
#else
	gammaProp.absValue = 0.5; //(0.5 - 4.0)
#endif
	gammaProp.autoManualMode = false;
	gammaProp.onOff = true;
	gammaProp.absControl = true;
	CheckError(cam.SetProperty(&gammaProp));
	CheckError(cam.GetProperty(&gammaProp));
	cout << "gamma = " << gammaProp.absValue << endl;

	const int WIDTH = min(floor(sqrt(maxPacketSize * 1000)), (double)min(imageInfo.maxWidth, imageInfo.maxHeight));
	const int HEIGHT = WIDTH;
	GigEImageSettings imageSettings;
	imageSettings.width = (WIDTH/4) * 4;  //Must be of intervals of 4
	imageSettings.height = (HEIGHT/2) * 2; //Must be of intervals of 2
	imageSettings.offsetX = ((imageInfo.maxWidth - WIDTH) / 2) / 4 * 4; //Must be of intervals of 4
	imageSettings.offsetY = ((imageInfo.maxHeight - HEIGHT) / 2) / 2 * 2; //Must be of intervals of 2
#ifdef COLOR
	imageSettings.pixelFormat = PIXEL_FORMAT_RGB8;
#else
	imageSettings.pixelFormat = PIXEL_FORMAT_MONO8;
#endif
    // Set the settings to the camera
	CheckError(cam.SetGigEImageSettings(&imageSettings));
	CheckError(cam.GetGigEImageSettings(&imageSettings));
	cv::Size imgSize(imageSettings.width, imageSettings.height);
	cout << "Image size = " << imgSize << endl; 	 
	cout << "Image X-offset = " << imageSettings.offsetX << ", Y-offset = " << imageSettings.offsetY << endl;
	cout << "Pixel format = " << imageSettings.pixelFormat << endl;
	
	// Start capturing images
    CheckError(cam.StartCapture());
    // Retrieve frame rate property
    Property frameProp;
    frameProp.type = FRAME_RATE;
    CheckError(cam.GetProperty( &frameProp ));
    cout << "Frame rate is " << fixed << setprecision(2) << frameProp.absValue << " fps" << endl; 

	
	Image rawImage;  
	cv::Mat lastFrame, lastStableFrame;
	bool isProcessed = false;
	int numUnchanged = 0;

	Room lastLighting = UNDEFINED;

	vector<RoomPhotos> vecRP = prepareData();

#ifdef SAVE
	deque<string> records;
	const cv::Size SAVE_FRAME_SIZE = cv::Size(480, 680);
	cv::VideoWriter outputVideo;
	int num_frames = 0;
	int MAX_FRAMES = 24 * 60; // 1 min record
	int CODEC = cv::VideoWriter::fourcc('F', 'M', 'P', '4');
	time_t curr_time = time(NULL);
	time_t prev_time = curr_time;
	string record = generateRecordName();
	outputVideo.open(record, CODEC, 24, SAVE_FRAME_SIZE, true);
	records.push_back(record);
	if (!outputVideo.isOpened())
	{
		cout << "Could not open record for writing... exiting" << endl;
		system("pause");
		exit(-1);
	}
#endif

    // Retrieve an image
	cv::Mat NowFrame;
	cv::Mat background;

	CheckError(cam.RetrieveBuffer( &rawImage ));
	// Get the raw image dimensions
	PixelFormat pixFormat;
	unsigned int rows, cols, stride;
	rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );
	cout << "Pixel format = " << pixFormat << endl;
#ifdef COLOR
	NowFrame = cv::Mat(rows, cols, CV_8UC3, rawImage.GetData()).clone();
	cv::cvtColor(NowFrame, NowFrame, CV_RGB2BGR);
#else
	NowFrame = cv::Mat(rows, cols, CV_8UC1, rawImage.GetData()).clone();
#endif

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
	
	cv::Mat bg_mask;
	cv::Ptr<cv::BackgroundSubtractor> p_mog2 = cv::createBackgroundSubtractorMOG2();	

	const int MIN_UNCHANGED = 3 * frameProp.absValue;
	const float DIFF_THRESH = 3.0;
	vector<cv::Mat> stableFrames(MIN_UNCHANGED);
	cv::Mat prev_rect_mask;
	cv::Mat prevFrame;
	vector<ColorRect> prev_rects;

	//TODO: Set this parameter
	const float MAX_MOVE = 10;
	vector<cv::Point2f> ROI_PTS2f = getROIPts2f();

	cv::Scalar CHAIR_COLOR1(0, 15, 0);
	cv::Scalar CHAIR_COLOR2(40, 90, 75);
	cv::Scalar TABLE_COLOR1(0, 0, 0);
	cv::Scalar TABLE_COLOR2(50, 20, 30);
	cv::Scalar TABLE_COLOR3(120, 15, 50);
	cv::Scalar TABLE_COLOR4(130, 45, 90);
	vector<tuple<cv::Scalar, cv::Scalar>> CHAIR_COLORS;
	CHAIR_COLORS.push_back(tuple<cv::Scalar, cv::Scalar>(CHAIR_COLOR1, CHAIR_COLOR2));
	vector<tuple<cv::Scalar, cv::Scalar>> TABLE_COLORS;
	TABLE_COLORS.push_back(tuple<cv::Scalar, cv::Scalar>(TABLE_COLOR1, TABLE_COLOR2));
	TABLE_COLORS.push_back(tuple<cv::Scalar, cv::Scalar>(TABLE_COLOR3, TABLE_COLOR4));
/*
	while(true)
	{
		CheckError(cam.RetrieveBuffer(&rawImage));
		cv::Mat f = cv::Mat(rows, cols, CV_8UC3, rawImage.GetData());
		cv::cvtColor(f, f, CV_RGB2BGR);
		cv::Mat rectFrame = drawColorRects(findColor(f, TABLE_COLOR1, TABLE_COLOR2), f.size());
		show("rects", rectFrame);
		cv::waitKey(500);
	}
*/

	while (true)
    {
	 	CheckError(cam.RetrieveBuffer( &rawImage ));
#ifdef COLOR
		NowFrame = cv::Mat(rows, cols, CV_8UC3, rawImage.GetData()).clone();
		cv::cvtColor(NowFrame, NowFrame, CV_RGB2BGR);
#else
		NowFrame = cv::Mat(rows, cols, CV_8UC1, rawImage.GetData()).clone();
#endif

		cv::Mat roiFrame = NowFrame.clone();
		
		p_mog2->apply(NowFrame, bg_mask);
		//show("bg_mask", bg_mask);
	
		//TODO: Get background under different illumination
/*
		if (!prev_rect_mask.empty() && !prevFrame.empty())
		{
			int DILATE_SIZE = 3;
			cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DILATE_SIZE, DILATE_SIZE), cv::Point(DILATE_SIZE / 2, DILATE_SIZE / 2));
			cv::dilate(prev_rect_mask, prev_rect_mask, element);
			//cout << "prev size = " << prevFrame.size() << ", prev_mask size = " << prev_rect_mask.size() << ", bg size = " << background.size() << endl;
			cv::inpaint(prevFrame, prev_rect_mask, background, 3, cv::INPAINT_TELEA);
		}
*/
		vector<ColorRect> chair_rects = findColor(NowFrame, CHAIR_COLOR1, CHAIR_COLOR2, cv::Scalar(0, 0, 255));// = findTexture(NowFrame, textures); 
		vector<ColorRect> table_rects = findColor(NowFrame, TABLE_COLOR1, TABLE_COLOR2, cv::Scalar(255, 0, 0));// = findRect(NowFrame, background);
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
					if (cv::norm(prev_rect->rect.center - now_rect->rect.center) < MAX_MOVE)
					{
						foundMatch = true;
						if (cv::pointPolygonTest(ROI_PTS2f, now_rect->rect.center, true) > 0)
						{	
							true_rects.push_back(*now_rect);
							//now_rect = now_rects.erase(now_rect); //Remove rect from future matching and correct now_rect iterator
						}
						break;
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
							setSound(audio, nowLighting);
#ifdef CURL
							setLight(curl, nowLighting);
#endif
						}
						else
							cout << "Equal to lastLighting" << endl;

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
					setSound(audio, nowLighting);
	#ifdef CURL
					setLight(curl, nowLighting);
	#endif
					isProcessed = true;
				}	
			}
		}

		lastFrame = combinedFrame.clone();
		cv::cvtColor(lastFrame.clone(), prev_rect_mask, CV_BGR2GRAY, 1);
 		prevFrame = NowFrame.clone();


#ifdef SAVE
		curr_time = time(NULL);
		if (curr_time - prev_time >= 1)
		{
			cv::Mat saveFrame;
			cv::resize(NowFrame, saveFrame, SAVE_FRAME_SIZE);
			cv::putText(saveFrame, getCurrentTime(), cv::Point(0, saveFrame.rows), CV_FONT_HERSHEY_PLAIN, 10, cv::Scalar(255, 255, 255), 10);
			outputVideo.write(saveFrame);
			prev_time = curr_time;
			num_frames++;
			if (num_frames > MAX_FRAMES)
			{
				string record = generateRecordName();
				outputVideo.open(record, CODEC, 24, SAVE_FRAME_SIZE, false);
				records.push_back(record);
				if (records.size() > 10)
				{
					remove(records[0].c_str());
					records.pop_front();
				}
				num_frames = 0;
			}
		}
#endif
        if(cv::waitKey(300) == 27) break;
    }
    cout << endl;
	cout << "Finished grabbing images" << endl; 

    // Stop capturing images
	CheckError(cam.StopCapture());  

    // Disconnect the camera
    CheckError(cam.Disconnect()); 

//	system("pause");
	return 0;
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
	// b) encode rectcosAngle positions and orientations and compare
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
	if (maxRP.score < 0.6)
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