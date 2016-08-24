#include "helper.h"
#include "CmFile.h"

#define MIN_ROOM_SCORE 0.4

vector<tuple<cv::Scalar, cv::Scalar>>CHAIR_COLORS;
vector<tuple<cv::Scalar, cv::Scalar>>TABLE_COLORS;

void initializeColors()
{
	/*
	cv::Scalar TABLE_COLOR1_1800(0, 100, 0);
	cv::Scalar TABLE_COLOR2_1800(35, 175, 100);
	cv::Scalar CHAIR_COLOR1_1800(0, 50, 0);
	cv::Scalar CHAIR_COLOR2_1800(35, 125, 100);
	*/

	cv::Scalar TABLE_COLOR1_1900(0, 0, 0);
	cv::Scalar TABLE_COLOR2_1900(50, 75, 150);
	cv::Scalar CHAIR_COLOR1_1900(50, 0, 0);
	cv::Scalar CHAIR_COLOR2_1900(125, 75, 150);
	//Living
	cv::Scalar LIVING_TABLE_COLOR1(0, 125, 100);
	cv::Scalar LIVING_TABLE_COLOR2(10, 200, 225);
	cv::Scalar LIVING_CHAIR_COLOR1(0, 125, 75);
	cv::Scalar LIVING_CHAIR_COLOR2(15, 175, 175);

	//Dining
	cv::Scalar DINING_TABLE_COLOR1(0, 125, 125);
	cv::Scalar DINING_TABLE_COLOR2(10, 200, 225);
	cv::Scalar DINING_CHAIR_COLOR1(0, 100, 75);
	cv::Scalar DINING_CHAIR_COLOR2(10, 175, 175);

	//Study
	cv::Scalar STUDY_TABLE_COLOR1(0, 50, 0);
	cv::Scalar STUDY_TABLE_COLOR2(25, 100, 200);
	cv::Scalar STUDY_CHAIR_COLOR1(150, 0, 50);
	cv::Scalar STUDY_CHAIR_COLOR2(25, 50, 150);

	CHAIR_COLORS = vector<tuple<cv::Scalar, cv::Scalar>>(NUM_ROOMS);
	TABLE_COLORS = vector<tuple<cv::Scalar, cv::Scalar>>(NUM_ROOMS);
	CHAIR_COLORS[LIVING] = tuple<cv::Scalar, cv::Scalar>(LIVING_CHAIR_COLOR1, LIVING_CHAIR_COLOR2);
	TABLE_COLORS[LIVING] = tuple<cv::Scalar, cv::Scalar>(LIVING_TABLE_COLOR1, LIVING_TABLE_COLOR2);
	CHAIR_COLORS[UNDEFINED] = tuple<cv::Scalar, cv::Scalar>(LIVING_CHAIR_COLOR1, LIVING_CHAIR_COLOR2);
	TABLE_COLORS[UNDEFINED] = tuple<cv::Scalar, cv::Scalar>(LIVING_TABLE_COLOR1, LIVING_TABLE_COLOR2);
	CHAIR_COLORS[DINING] = tuple<cv::Scalar, cv::Scalar>(DINING_CHAIR_COLOR1, DINING_CHAIR_COLOR2);
	TABLE_COLORS[DINING] = tuple<cv::Scalar, cv::Scalar>(DINING_TABLE_COLOR1, DINING_TABLE_COLOR2);
	CHAIR_COLORS[STUDY] = tuple<cv::Scalar, cv::Scalar>(STUDY_CHAIR_COLOR1, STUDY_CHAIR_COLOR2);
	TABLE_COLORS[STUDY] = tuple<cv::Scalar, cv::Scalar>(STUDY_TABLE_COLOR1, STUDY_TABLE_COLOR2);
	CHAIR_COLORS[ON] = tuple<cv::Scalar, cv::Scalar>(CHAIR_COLOR1_1900, CHAIR_COLOR2_1900);
	TABLE_COLORS[ON] = tuple<cv::Scalar, cv::Scalar>(TABLE_COLOR1_1900, TABLE_COLOR2_1900);
	CHAIR_COLORS[OFF] = tuple<cv::Scalar, cv::Scalar>(CHAIR_COLOR1_1900, CHAIR_COLOR2_1900);
	TABLE_COLORS[OFF] = tuple<cv::Scalar, cv::Scalar>(TABLE_COLOR1_1900, TABLE_COLOR2_1900);
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

vector<RoomPhotos> prepareData()
{
	//Use full resolution for rectangle extraction, resize results
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
Room getSimilarRoom(cv::Mat &img, vector<RoomPhotos> &vecRP)
{
	
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
	if (maxRP.score < MIN_ROOM_SCORE)
	{
		return Room::UNDEFINED;
	}
	else
	{
		show("most_similar", cv::Mat(maxRP.img.get_MAT()));
		return maxRP.room;
	}
}

//TODO: Check values
double calculateDifference(cv::Mat &img1, cv::Mat &img2)
{
	cv::Mat diff;
    cv::absdiff(img1, img2, diff);
	//cv::bitwise_xor(img1, img2, diff);
    cv::Mat diff_rgb[3];
    cv::split(diff, diff_rgb);
    cv::Mat total_diff = cv::max(diff_rgb[0], diff_rgb[1]);
    total_diff = cv::max(total_diff, diff_rgb[2]);
	return cv::mean(total_diff)[0] / 255 * 100;
}
