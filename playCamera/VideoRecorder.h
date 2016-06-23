#pragma once
#include "stdafx.h"

class VideoRecorder
{
public:
	VideoRecorder(GigECamera cam, cv::Size image_size);
	~VideoRecorder(void);
	string generateRecordName();
	void recordCont();
private:
	std::thread m_recordRunner;
	cv::Size SAVE_FRAME_SIZE;
	int MAX_FRAMES;
	int CODEC;
	cv::Size m_image_size;
	deque<string> m_records;
	time_t curr_time;
	time_t prev_time;
	int num_frames;
	GigECamera m_cam;
	cv::VideoWriter m_outputVideo;
	bool isExiting;
};

