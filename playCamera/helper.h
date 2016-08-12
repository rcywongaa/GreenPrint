#include "stdafx.h"

vector<cv::Mat> generateTemplates(cv::Mat input);
bool initCurl(CURL* &curl);
void sendUrl(CURL* curl, int gp, int r, int g, int b);
void setLight(CURL* curl, Room room);
vector<RoomPhotos> prepareData();
double calcScore(cv::Mat img, cv::Mat train);
Room getSimilarRoom(cv::Mat &img, vector<RoomPhotos> &vecRP);
double calculateDifference(cv::Mat &img1, cv::Mat &img2);

/********** CONSTANTS **********/
extern vector<tuple<cv::Scalar, cv::Scalar>> CHAIR_COLORS;
extern vector<tuple<cv::Scalar, cv::Scalar>> TABLE_COLORS;
