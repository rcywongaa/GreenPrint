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

vector<tuple<cv::Scalar, cv::Scalar>> CHAIR_COLORS(5);
vector<tuple<cv::Scalar, cv::Scalar>> TABLE_COLORS(5);
CHAIR_COLORS[LIVING] = tuple<cv::Scalar, cv::Scalar>(LIVING_CHAIR_COLOR1, LIVING_CHAIR_COLOR2);
TABLE_COLORS[LIVING] = tuple<cv::Scalar, cv::Scalar>(LIVING_TABLE_COLOR1, LIVING_TABLE_COLOR2);
CHAIR_COLORS[UNDEFINED] = tuple<cv::Scalar, cv::Scalar>(LIVING_CHAIR_COLOR1, LIVING_CHAIR_COLOR2);
TABLE_COLORS[UNDEFINED] = tuple<cv::Scalar, cv::Scalar>(LIVING_TABLE_COLOR1, LIVING_TABLE_COLOR2);
CHAIR_COLORS[DINING] = tuple<cv::Scalar, cv::Scalar>(DINING_CHAIR_COLOR1, DINING_CHAIR_COLOR2);
TABLE_COLORS[DINING] = tuple<cv::Scalar, cv::Scalar>(DINING_TABLE_COLOR1, DINING_TABLE_COLOR2);
CHAIR_COLORS[STUDY] = tuple<cv::Scalar, cv::Scalar>(STUDY_CHAIR_COLOR1, STUDY_CHAIR_COLOR2);
TABLE_COLORS[STUDY] = tuple<cv::Scalar, cv::Scalar>(STUDY_TABLE_COLOR1, STUDY_TABLE_COLOR2);
CHAIR_COLORS[SIX] = tuple<cv::Scalar, cv::Scalar>(CHAIR_COLOR1_1800, CHAIR_COLOR2_1800);
TABLE_COLORS[SIX] = tuple<cv::Scalar, cv::Scalar>(TABLE_COLOR1_1800, TABLE_COLOR2_1800);
