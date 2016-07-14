#include "SoundController.h"
#include "sndfile.h"

/*
double GetMediaDuration(string MediaFilename)
{
    double duration = 0.0;
    using (FileStream fs = File.OpenRead(MediaFilename))
    {
        Mp3Frame frame = Mp3Frame.LoadFromStream(fs);
        if (frame != null)
        {
            _sampleFrequency = (uint)frame.SampleRate;
        }
        while (frame != null)
        {
            if (frame.ChannelMode == ChannelMode.Mono)
            {
                duration += (double)frame.SampleCount * 2.0 / (double)frame.SampleRate;
            }
            else
            {
                duration += (double)frame.SampleCount * 4.0 / (double)frame.SampleRate;
            }
            frame = Mp3Frame.LoadFromStream(fs);
        }
    }
    return duration;
}
*/

SoundController::SoundController()
{
	isExiting = false;
	CoInitialize(NULL);
	IMMDeviceEnumerator *deviceEnumerator = NULL;
	HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL, CLSCTX_INPROC_SERVER, __uuidof(IMMDeviceEnumerator), (LPVOID *)&deviceEnumerator);
	IMMDevice *defaultDevice = NULL;

	hr = deviceEnumerator->GetDefaultAudioEndpoint(eRender, eConsole, &defaultDevice);
	deviceEnumerator->Release();
	deviceEnumerator = NULL;

	hr = defaultDevice->Activate(__uuidof(IAudioEndpointVolume), CLSCTX_INPROC_SERVER, NULL, (LPVOID *)&m_audio_endpoint);
	defaultDevice->Release();
	defaultDevice = NULL;

	if (m_audio_endpoint == NULL)
		cout << "Failed to get audio endpoint" << endl;

	m_room = UNDEFINED;
	m_isPlaying = false;
	NUM_SOUNDS[LIVING] = NUM_LIVING_SOUNDS;
	NUM_SOUNDS[DINING] = NUM_DINING_SOUNDS;
	NUM_SOUNDS[STUDY] = NUM_STUDY_SOUNDS;
}

SoundController::~SoundController()
{
	isExiting = true;
	while (m_playSoundRunner.joinable() == false);
	m_playSoundRunner.join();
}

void SoundController::setRoom(Room room)
{
	m_room = room;
	while (m_isPlaying);
	if (m_playSoundRunner.joinable())
		m_playSoundRunner.join();
	m_playSoundRunner = std::thread([&] {playSoundCont();});
}

float SoundController::playSound(Room room)
{
	float duration = 0;
	if (room < UNDEFINED)
	{
		SNDFILE* file;
		SF_INFO info;
		int idx = rand() % NUM_SOUNDS[room] + 1;
		string filename = "D:/sound/" + getString(room) + "/" + to_string(idx) + ".wav";
		if (!(file = sf_open(filename.c_str(), SFM_READ, &info)))
			cout << "Cannot open wav file" << endl;
		duration = info.frames / info.samplerate;
		PlaySound(std::wstring(filename.begin(), filename.end()).c_str(), NULL, SND_ASYNC);
		//PlaySound(L"D:/sound/one_summers_day.wav", NULL, SND_LOOP | SND_ASYNC);
		cout << "Playing " + filename << "(:" << duration << ")" << endl;
	}
	else
	{
		PlaySound(NULL, NULL, SND_ASYNC); // to stop music
		//cout << "Stop" << endl;
	}
	return duration;
}

void SoundController::playSoundCont()
{
	m_isPlaying = true;
	Room curr_room = m_room;
	time_t start_time = time(NULL);
	float duration = 0;
	duration = playSound(curr_room);
	fadeIn();
	while (true)
	{
		if (isExiting) break;
		time_t curr_time = time(NULL);
		//cout << "Playtime = " << curr_time - start_time << endl;
		if (curr_time - start_time > duration) // duration of soundtrack
		{
			start_time = time(NULL);
			duration = playSound(curr_room);
		}

		if (curr_room != m_room) // room changed
		{
			break;
		}
		Sleep(1000);
	}
	fadeOut();
	playSound(UNDEFINED);
	m_isPlaying = false;
}

void SoundController::fadeIn()
{
	for (int i = 0; i <= 100; i++)
	{
		m_audio_endpoint->SetMasterVolumeLevelScalar(i/100.0 * MAX_VOL / 100.0, NULL);
		Sleep(50);
	}
}

void SoundController::fadeOut()
{
	for (int i = 100; i >= 0; i--)
	{
		m_audio_endpoint->SetMasterVolumeLevelScalar(i/100.0 * MAX_VOL / 100.0, NULL);
		Sleep(50);
	}
}