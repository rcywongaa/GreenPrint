#pragma once
#include "stdafx.h"
#include "MultithreadCam.h"

class VideoRecorder
{
public:
	VideoRecorder(MultithreadCam* cam);
	~VideoRecorder(void);
	string generateRecordName();
	void recordCont();
private:
	std::thread m_recordRunner;
	int FONT_SIZE;
	int FRAME_RATE;
	cv::Size SAVE_FRAME_SIZE;
	int DURATION;
	int MAX_NUM_RECORDS;
	int CODEC;
	bool ISCOLOR;
	deque<string> m_records;
	time_t curr_time;
	time_t prev_time;
	int num_frames;
	MultithreadCam* m_cam;
	cv::VideoWriter m_outputVideo;
	bool isExiting;
};

