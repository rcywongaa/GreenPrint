// playCamera.cpp : Defines the entry point for the console application.
//

/*
TODO:
Capture backgrounds under different lighting conditions
New training layouts
*/

#include "stdafx.h"
#include "CmFile.h"



struct RoomPhotos
{
	CImg<uchar>img;
	double score;
	string  type;
};

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

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

void CheckError( Error error ){
	if (error != PGRERROR_OK)
    {
		//Ignore image consistency error for now...
        //PrintError( error );
        //exit(-1);
    }
}

void show(string window, cv::Mat img)
{
	cv::Mat resized;
	//cv::resize(img, resized, cv::Size(320, 240));
	cv::resize(img, resized, cv::Size(640, 480));
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

struct Texture {
	cv::Scalar color;
	cv::Mat image;
	vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	int max_instances;
	int type;
};

cv::Mat findTexture(cv::Mat input, vector<Texture> textures)
{
	/********** Feature detection to identify box **********/
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
	//TODO: Deal with multiple object detection
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
			cout << "Not enough matches: " << type << endl;
			continue;
		}
		cv::Mat matchFrame;
		cv::drawMatches(input, get<0>(matched_keypoints[type]), textures[type].image, get<1>(matched_keypoints[type]), good_matches, matchFrame);
		show("match - " + to_string(type), matchFrame);
		//TODO: partition?
		//TODO: clustering
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
			cout << width << ", " << height << endl;
			vector<cv::Point2f> tex_bb;
			tex_bb.push_back(cv::Point2f(0, 0));
			tex_bb.push_back(cv::Point2f(width, 0));
			tex_bb.push_back(cv::Point2f(width, height));
			tex_bb.push_back(cv::Point2f(0, height));
			if (!homography.empty())
			{
				cv::perspectiveTransform(tex_bb, object_bb, homography);
				cout << homography << endl;
				cout << object_bb << endl;
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
				}
				else
					cout << "Bad homography matrix" << endl;
			}
			else
				cout << "Homography failed" << endl;
		}
		show("cluster - " + to_string(type), clusterFrame);
		show("homo - " + to_string(type), homoFrame);
	}
	show("tex", texFrame);
	return texFrame;
}

cv::Mat findRect(cv::Mat input, cv::Mat background)
{
	if (background.size() != input.size())
	{
		cout << "Warning: Auto-resizing background" << endl;
		cv::resize(background, background, input.size());
	}
	show("input", input);
	show("background", background);
	cv::Mat filteredFrame;
	//TODO: Perhaps use absdiff?
	cv::subtract(input, background, filteredFrame);
	filteredFrame = cv::abs(filteredFrame);
	show("subtracted", filteredFrame);

	//cv::threshold(filteredFrame, filteredFrame, 100, 255, CV_THRESH_BINARY); //Emphasize all edges
	cv::medianBlur(filteredFrame, filteredFrame, 11);
	cv::adaptiveThreshold(filteredFrame, filteredFrame, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, -10);
	show("filtered", filteredFrame);

	cv::Mat edgeFrame;
	cv::Canny(filteredFrame, edgeFrame, 0, 300, 3, true);
	int DILATE_SIZE = 5;
	//cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DILATE_SIZE, DILATE_SIZE), cv::Point(DILATE_SIZE / 2, DILATE_SIZE / 2));
	cv::dilate(edgeFrame, edgeFrame, element); // Fill holes in contours
	//TODO: Do line thinning here
	show("edges", edgeFrame);

	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> contour_hierarchy;
	cv::findContours(edgeFrame, contours, contour_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cv::Mat contourFrame = cv::Mat::zeros(input.size(), input.type());
	cv::drawContours(contourFrame, contours, -1, cv::Scalar(255, 255, 255));
	show("contours", contourFrame);

	cv::Mat rectFrame = cv::Mat::zeros(input.size(), input.type());
	//cout << contours.size() << endl;
	cv::Mat polyFrame = cv::Mat::zeros(input.size(), input.type());
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
					//TODO: Choose fill color according to furniture type
					cv::fillConvexPoly(rectFrame, &pts[0], 4, cv::Scalar(255, 255, 255));
				}
			}
		}
	}
	show("polygon", polyFrame);
	show("rect", rectFrame);
	//cv::waitKey();
	return rectFrame;
}

vector<cv::Mat> generateTemplates(cv::Mat input)
{
	float PAD_SCALE = 1.5;
	//Consider coloring rectcosAngles to prevent matching whitespace
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

void setSound(IAudioEndpointVolume *audio, string type)
{
	for (int i = 100; i >= 0; i--)
	{
		audio->SetMasterVolumeLevelScalar(i/100.0, NULL);
		Sleep(50);
	}
	
	if (type == "living")
		PlaySound(L"D:/Sound/canon_in_d.wav", NULL, SND_LOOP | SND_ASYNC);
	else if (type == "dining")
		PlaySound(L"D:/Sound/one_summers_day.wav", NULL, SND_LOOP | SND_ASYNC);
	else if (type == "study")
		PlaySound(L"D:/Sound/ocean.wav", NULL, SND_LOOP | SND_ASYNC);
	
	//PlaySound(NULL, NULL, SND_LOOP | SND_ASYNC); // to stop music

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
	int CLUSTER_IDX = 3;
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

void setLight(CURL* curl, string style)
{
	int CLUSTER_IDX = 3;
	unsigned char light_color[3] = {0, 0, 0};
	if (style == "off")
	{
		cout << "Switching off..." << endl;
		light_color[0] = 0;
		light_color[1] = 0;
		light_color[2] = 0;
	}
	else if (style == "dining")
	{
		cout << "Changing to dining lights..." << endl;
		light_color[0] = 255;
		light_color[1] = 0;
		light_color[2] = 0;
	}
	else if (style == "living")
	{
		cout << "Changing to living lights..." << endl;
		light_color[0] = 0;
		light_color[1] = 255;
		light_color[2] = 0;
	}
	else if (style == "study")
	{
		cout << "Changing to study lights..." << endl;
		light_color[0] = 0;
		light_color[1] = 0;
		light_color[2] = 255;
	}
	else
	{
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

string generateRecordName()
{
	time_t curr_time = time(NULL);
	char time_char[128];
	strftime(time_char, 128, "%Y-%m-%d-%H%M%S", localtime(&curr_time));
	string time_str(time_char);
	string filename = "record_" + time_str + ".avi";
	return filename;
}

vector<RoomPhotos> prepareData(cv::Size &imgsize);
string outputInfo(cv::Mat &img, vector<RoomPhotos> &vecRP);
double calculateDifference(cv::Mat &img1, cv::Mat &img2);

int _tmain(int argc, _TCHAR* argv[])
{
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
		show("keypoints - " + to_string(type), keypointFrame);
		textures[type].max_instances = 2;
		textures[type].color = cv::Scalar(255, 0, 0);
		textures[type].type = type;
	}

	cv::Mat tex_test = cv::imread("D:/tex_test.png");
	findTexture(tex_test, textures);
	cv::waitKey();

	/*
	vector<RoomPhotos> testData = prepareData(cv::Size(640, 480));
	cv::Mat test = cv::imread("D:/failed1.jpg", 0);
	outputInfo(test, testData);
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

	Camera cam;
    // Connect to a camera
	CheckError(cam.Connect(&guid));

	// Get the camera information
    CameraInfo camInfo;
	CheckError(cam.GetCameraInfo(&camInfo));

    PrintCameraInfo(&camInfo);   

	const Mode k_fmt7Mode = MODE_2;
	const PixelFormat k_fmt7PixFmt = PIXEL_FORMAT_BGR;

	 // Query for available Format 7 modes
    Format7Info fmt7Info;
    bool supported;
    fmt7Info.mode = k_fmt7Mode;
	CheckError(cam.GetFormat7Info( &fmt7Info, &supported ));

    PrintFormat7Capabilities( fmt7Info );
    if ( (k_fmt7PixFmt & fmt7Info.pixelFormatBitField) == 0 )
    {
        // Pixel format not supported!
		cout << "Pixel format is not supported" << endl; 
        return -1;
    }
    
    Format7ImageSettings fmt7ImageSettings;
    fmt7ImageSettings.mode = k_fmt7Mode;
    fmt7ImageSettings.offsetX = 0;
    fmt7ImageSettings.offsetY = 0;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    fmt7ImageSettings.pixelFormat = k_fmt7PixFmt;

    bool valid;
    Format7PacketInfo fmt7PacketInfo;

    // Validate the settings to make sure that they are valid
	CheckError(cam.ValidateFormat7Settings(
        &fmt7ImageSettings,
        &valid,
        &fmt7PacketInfo )
		);

    if ( !valid )
    {
        // Settings are not valid
		cout << "Format7 settings are not valid" << endl; 
        return -1;
    }

    // Set the settings to the camera
	CheckError( cam.SetFormat7Configuration(
        &fmt7ImageSettings,
        fmt7PacketInfo.recommendedBytesPerPacket )
		);

	// Start capturing images
    CheckError(cam.StartCapture());

    // Retrieve frame rate property
    Property frmRate;
    frmRate.type = FRAME_RATE;
    CheckError(cam.GetProperty( &frmRate ));
    cout << "Frame rate is " << fixed << setprecision(2) << frmRate.absValue << " fps" << endl; 

	Image rawImage;  
	cv::Mat lastFrame, lastStableFrame;
	bool isProcessed = false;
	int numUnchanged = 0;

	string lastLighting = "";

	cv::Size imgSize( fmt7Info.maxWidth / 3, fmt7Info.maxHeight / 3 );
	vector<RoomPhotos> vecRP = prepareData(imgSize);

#ifdef SAVE
	cv::VideoWriter outputVideo;
	int num_frames = 0;
	int MAX_FRAMES = 24 * 60; // 1 min record
	int CODEC = cv::VideoWriter::fourcc('F', 'M', 'P', '4');
	time_t curr_time = time(NULL);
	time_t prev_time = curr_time;
	outputVideo.open(generateRecordName(), CODEC, 24, imgSize, false);
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
	NowFrame = cv::Mat(rows, cols, CV_8UC1, rawImage.GetData()).clone();

	// If background image exists and less than 1 hour old, use it
	struct stat st;
	if (stat("background.jpg", &st) == 0 && time(NULL) - st.st_mtime < 12*60*60)
	{
		background = cv::imread("background.jpg", 0);
	}
	else
	{
		background = NowFrame.clone();
		cv::imwrite("background.jpg", background);
	}

	const int MIN_UNCHANGED = 24;
	vector<cv::Mat> stableFrames(MIN_UNCHANGED);
	while (true)
    {
	 	CheckError(cam.RetrieveBuffer( &rawImage ));
		NowFrame = cv::Mat(rows, cols, CV_8UC1, rawImage.GetData()).clone(); //Prevent NowFrame from changing unexpectedly
		cv::Mat rectFrame = findRect(NowFrame, background);
		cv::resize(rectFrame, rectFrame, imgSize);

		if (cv::countNonZero(rectFrame) > 0) //Prevent phash division by zero error
		{
			if (!lastFrame.empty())
			{	
				double diff = calculateDifference(rectFrame, lastFrame);
				cout << diff << endl;
				if(diff < 0.5){
					numUnchanged++;
					stableFrames[numUnchanged % MIN_UNCHANGED] = rectFrame;
				}else{
					stableFrames.clear();
					numUnchanged = 0;
					isProcessed = false;
					cout << "Change detected: " << diff << endl;
				}
			}

			// needProcess
			if (numUnchanged >= MIN_UNCHANGED && isProcessed == false)
			{
				cv::Mat averageFrame = cv::Mat::zeros(rectFrame.size(), CV_8UC1);
				for (auto frame : stableFrames)
				{
					averageFrame += frame / stableFrames.size();
				}
				show("average", averageFrame);
				if (!lastStableFrame.empty())
				{
					double diff = calculateDifference(lastStableFrame,averageFrame);
					if ( diff > 0.5)
					{
						string nowLighting = outputInfo(averageFrame, vecRP);
						if (lastLighting != nowLighting)
						{
							cout<<nowLighting<<endl;
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
					string nowLighting = outputInfo(averageFrame, vecRP);
					lastLighting = nowLighting;
					cout<<nowLighting<<endl;
					setSound(audio, nowLighting);
#ifdef CURL
					setLight(curl, nowLighting);
#endif
					isProcessed = true;
				}	
			}
		}

		lastFrame = rectFrame.clone();

#ifdef SAVE
		curr_time = time(NULL);
		if (curr_time - prev_time >= 1)
		{
			cv::Mat saveFrame;
			cv::resize(NowFrame, saveFrame, imgSize);
			outputVideo.write(saveFrame);
			prev_time = curr_time;
			num_frames++;
			if (num_frames > MAX_FRAMES)
			{
				outputVideo.open(generateRecordName(), CODEC, 24, imgSize, false);
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


vector<RoomPhotos> prepareData(cv::Size &imgsize){
	//Use full resolution for rectcosAngle extraction, resize results
	vector<RoomPhotos> vecRP;
	vecRP.reserve(100);

	string wkdir = "D:/TrainData/";
	string imgNameW1 = wkdir + "dining/*.jpg";
	string imgNameW2 = wkdir + "living/*.jpg";
	string imgNameW3 = wkdir + "study/*.jpg";
	string background = wkdir + "background.jpg";
	cv::Mat bg_gray = cv::imread(background, 0);
	vecS namesNE;
	int imgNum;
	struct stat st;

	//TODO: Create array of type strings to iterate over
	// dining
	imgNum = CmFile::GetNamesNE(imgNameW1, namesNE);
	for (int i = 0; i < imgNum; i++)
	{
		RoomPhotos temp;
		cv::Mat train;
		if (stat((wkdir + "dining/filtered/" + namesNE[i] + "_filtered.jpg").c_str(), &st) == 0)
		{
			train = cv::imread(wkdir + "dining/filtered/" + namesNE[i] + "_filtered.jpg", 0); 
			cv::resize(train, train,imgsize);
		}
		else
		{
			cv::Mat gray = cv::imread(wkdir + "dining/" + namesNE[i] + ".jpg", 0);
			train = findRect(gray, bg_gray);
			cv::resize(train, train,imgsize);
			cv::imwrite(wkdir + "dining/filtered/" + namesNE[i] + "_filtered.jpg", train);
		}
		temp.img = CImg<uchar>(train);
		temp.score = 0;
		temp.type = "dining";
		vecRP.push_back(temp);
		cout << "Added " + namesNE[i] << ":" << imgsize << endl;
	}


	// livin
	namesNE.clear();
	imgNum = CmFile::GetNamesNE(imgNameW2, namesNE);
	for (int i = 0; i < imgNum; i++)
	{
		RoomPhotos temp;
		cv::Mat train;
		if (stat((wkdir + "living/filtered/" + namesNE[i] + "_filtered.jpg").c_str(), &st) == 0)
		{
			train = cv::imread(wkdir + "living/filtered/" + namesNE[i] + "_filtered.jpg", 0); 
			cv::resize(train, train,imgsize);
		}
		else
		{
			cv::Mat gray = cv::imread(wkdir + "living/" + namesNE[i] + ".jpg", 0);
			train = findRect(gray, bg_gray);
			cv::resize(train, train,imgsize);
			cv::imwrite(wkdir + "living/filtered/" + namesNE[i] + "_filtered.jpg", train);
		}
		temp.img = CImg<uchar>(train);
		temp.score = 0;
		temp.type = "living";
		vecRP.push_back(temp);
		cout << "Added " + namesNE[i] << ":" << imgsize << endl;
	}

	// study
	namesNE.clear();
	imgNum = CmFile::GetNamesNE(imgNameW3, namesNE);
	for (int i = 0; i < imgNum; i++)
	{
		RoomPhotos temp;
		cv::Mat train;
		if (stat((wkdir + "study/filtered/" + namesNE[i] + "_filtered.jpg").c_str(), &st) == 0)
		{
			train = cv::imread(wkdir + "study/filtered/" + namesNE[i] + "_filtered.jpg", 0); 
			cv::resize(train, train,imgsize);
		}
		else
		{
			cv::Mat gray = cv::imread(wkdir + "study/" + namesNE[i] + ".jpg", 0);
			train = findRect(gray, bg_gray);
			cv::resize(train, train,imgsize);
			cv::imwrite(wkdir + "study/filtered/" + namesNE[i] + "_filtered.jpg", train);
		}
		temp.img = CImg<uchar>(train);
		temp.score = 0;
		temp.type = "study";
		vecRP.push_back(temp);
		cout << "Added " + namesNE[i] << ":" << imgsize << endl;
	}

	return vecRP;
}

double calcScore(cv::Mat img, cv::Mat train)
{
	// a) resize, center and run phash on different rotations
	// b) encode rectcosAngle positions and orientations and compare
	// c) template matching with multiscale multiorientation training data

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
		cv::threshold(result, result, 0, 255, cv::THRESH_TOZERO);
		cv::minMaxLoc(result, &this_min, &this_max, &min_pos, &max_pos);
		//imshow("tmpl", tmpl);
		//imshow("tm", result);
		//cv::waitKey();
		if (this_max > max)
		{
			max = this_max;
		}
	}
	//cv::waitKey();
	return max;
}


//Use normalized distance and normalized orientation to compare
string outputInfo(cv::Mat &img, vector<RoomPhotos> &vecRP){
	
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

	show("most_similar", cv::Mat(maxRP.img.get_MAT()));
	cout << "Score: " << maxRP.score << endl;
	if (maxRP.score < 0.6)
	{
		return(string("undefine type"));
	}else
	{
		return maxRP.type;
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