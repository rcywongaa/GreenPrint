#include "VideoRecorder.h"


VideoRecorder::VideoRecorder(GigECamera cam, cv::Size image_size)
{
	SAVE_FRAME_SIZE = cv::Size(480, 680);
	num_frames = 0;
	MAX_FRAMES = 24 * 60; // 1 min record
	CODEC = cv::VideoWriter::fourcc('F', 'M', 'P', '4');
	curr_time = time(NULL);
	prev_time = curr_time;
	string record = generateRecordName();
	m_outputVideo.open(record, CODEC, 24, SAVE_FRAME_SIZE, true);
	m_records.push_back(record);
	if (!m_outputVideo.isOpened())
	{
		cout << "Could not open record for writing... exiting" << endl;
		system("pause");
		exit(-1);
	}
	isExiting = false;
	m_cam = cam;
	m_image_size = image_size;
	m_recordRunner = std::thread([&] {recordCont();});
}


VideoRecorder::~VideoRecorder(void)
{
	isExiting = true;
	while(m_recordRunner.joinable() == false);
	m_recordRunner.join();
}

string VideoRecorder::generateRecordName()
{
	string time_str = getCurrentTime();
	string filename = "D:/recording/record_" + time_str + ".avi";
	return filename;
}

void VideoRecorder::recordCont()
{
	while(true)
	{
		if (isExiting) break;
		cv::Mat NowFrame;
		Image rawImage;
		if (CheckError(m_cam.RetrieveBuffer( &rawImage )) != 0)
			continue;
		NowFrame = cv::Mat(m_image_size, CV_8UC3, rawImage.GetData()).clone();
		cv::cvtColor(NowFrame, NowFrame, CV_RGB2BGR);
		curr_time = time(NULL);
		if (curr_time - prev_time >= 1)
		{
			cv::Mat saveFrame;
			cv::resize(NowFrame, saveFrame, SAVE_FRAME_SIZE);
			cv::putText(saveFrame, getCurrentTime(), cv::Point(0, saveFrame.rows), CV_FONT_HERSHEY_PLAIN, 10, cv::Scalar(255, 255, 255), 10);
			m_outputVideo.write(saveFrame);
			prev_time = curr_time;
			num_frames++;
			if (num_frames > MAX_FRAMES)
			{
				string record = generateRecordName();
				m_outputVideo.open(record, CODEC, 24, SAVE_FRAME_SIZE, false);
				m_records.push_back(record);
				if (m_records.size() > 10)
				{
					remove(m_records[0].c_str());
					m_records.pop_front();
				}
				num_frames = 0;
			}
		}
		Sleep(1000);
	}
}