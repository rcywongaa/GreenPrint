#include "VideoRecorder.h"

VideoRecorder::VideoRecorder(MultithreadCam* cam)
{
	SAVE_FRAME_SIZE = cv::Size(680, 680);
	num_frames = 0;
	FRAME_RATE = 2;
	DURATION = 24*60*60/FRAME_RATE; //seconds
	MAX_NUM_RECORDS = 365;
	FONT_SIZE = 2;
	ISCOLOR = true;
	CODEC = cv::VideoWriter::fourcc('F', 'M', 'P', '4');
	curr_time = time(NULL);
	prev_time = curr_time;
	string record = generateRecordName();
	m_outputVideo.open(record, CODEC, FRAME_RATE, SAVE_FRAME_SIZE, ISCOLOR);
	m_records.push_back(record);
	if (!m_outputVideo.isOpened())
	{
		cout << "Could not open record for writing... exiting" << endl;
		system("pause");
		exit(-1);
	}
	isExiting = false;
	m_cam = cam;
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
		//cout << "Saving frame... " << endl;
		curr_time = time(NULL);
		if (curr_time - prev_time >= 1)
		{
			cv::Mat NowFrame = m_cam->getImage();
			cv::cvtColor(NowFrame, NowFrame, CV_RGB2BGR);
			cv::Mat saveFrame;
			cv::resize(NowFrame, saveFrame, SAVE_FRAME_SIZE);
			cv::putText(saveFrame, getCurrentTime(), cv::Point(0, saveFrame.rows), CV_FONT_HERSHEY_PLAIN, FONT_SIZE, cv::Scalar(255, 255, 255), FONT_SIZE);
			m_outputVideo.write(saveFrame);
			prev_time = curr_time;
			num_frames++;
			if (num_frames > DURATION * FRAME_RATE)
			{
				string record = generateRecordName();
				m_outputVideo.open(record, CODEC, FRAME_RATE, SAVE_FRAME_SIZE, ISCOLOR);
				m_records.push_back(record);
				if (m_records.size() > MAX_NUM_RECORDS)
				{
					remove(m_records[0].c_str());
					m_records.pop_front();
				}
				num_frames = 0;
			}
		}
		Sleep(500);
	}
}