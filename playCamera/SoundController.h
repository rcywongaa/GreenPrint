#include "stdafx.h"

class SoundController
{
	public:
		SoundController();
		~SoundController();
		float playSound(Room room);
		void playSoundCont();
		void setRoom(Room room);
		void fadeIn();
		void fadeOut();
	private:
		std::mutex m_room_mtx;
		IAudioEndpointVolume* m_audio_endpoint;
		volatile Room m_room;
		volatile bool m_isPlaying;
		std::thread m_playSoundRunner;
		static const int MAX_VOL = 10; // Between 0 - 100
		static const int NUM_LIVING_SOUNDS = 29;
		static const int NUM_DINING_SOUNDS = 9;
		static const int NUM_STUDY_SOUNDS = 21;
		int NUM_SOUNDS[3];
		bool isExiting;
};

		