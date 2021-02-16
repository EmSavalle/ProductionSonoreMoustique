/**
*
* \brief This is the main source file of the example project 1 using 3D Tune-In Toolkit
* \date	April 2018
*

* \authors A. Rodríguez-Rivero, as part of the 3DI-DIANA Research Group (University of Malaga)

* \b Contact: A. Reyes-Lecuona as head of 3DI-DIANA Research Group (University of Malaga): areyes@uma.es
*
* \b Contributions: (additional authors/contributors can be added here)
*
* \b Project: 3DTI (3D-games for TUNing and lEarnINg about hearing aids) ||
* \b Website: http://3d-tune-in.eu/
*
* \b Copyright: University of Malaga - 2018
*
* \b Licence: GPLv3
*
* \b Acknowledgement: This project has received funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 644051
*
*/
#include "BasicSpatialisationRTAudio.h"
#include "../projects/vstudio/AudioFile.h"
#include <zmq.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include "AudioFile.h"
using namespace std::this_thread;
using namespace std;
using namespace std::chrono;
Moustique m;
#define SAMPLERATE 44100
bool saveAudio = true;
bool saveMouvement = false;
bool equalizedSound = false;
bool incrementalSpeed = true;
bool waitZMQ = false;
bool askPos = true;
int posxAudio = 0;
int posyAudio = 0;
int posxInitAudio = 20;
int posyInitAudio = 20;
void initialization(bool ask) {
    m.init(posxAudio, posyAudio, 0, ask, 0.5);
}
void updatePosition(int x, int y){
    if (!saveAudio) {
        m.setDestination(Common::CVector3(x, y, 0));
        m.authorizeMovement = true;

    }

}
void end() {
    m.stop();
}
int main()
{

    // 2. Set to (e.g.) two channels
    bufferSave.resize(2);

    // 3. Set number of samples per channel
    bufferSave[0].resize(100000);
    bufferSave[1].resize(100000);
    void* context = zmq_ctx_new();

    void* responder = zmq_socket(context, ZMQ_REP);
    int rc = zmq_bind(responder, "tcp://*:5555");
    int major = 0;
    int minor = 0;
    int patch = 0;
    zmq_version(&major, &minor, &patch);
    string st;
    // Connect to socket
    if (!saveAudio) {
        m.init(posxAudio, posyAudio, 0, askPos, 0.5);
        while (!m.ended) {
            if (m.waitForAuthorization) {

                cout << "New position " << m.position.x << "/" << m.position.y << "/" << m.position.z << "\n";
                if (waitZMQ) {
                    printf("waiting for message from server");
                    char buffer[100];
                    //Search for message
                    zmq_recv(responder, buffer, 10, 0);

                    int a_size = sizeof(buffer) / sizeof(char);

                    string s_a = convertToString(buffer, a_size);
                    printf("Received message");
                    std::string delimiter = "/";
                    vector<string> spl = split(s_a, delimiter);
                    cout << spl.at(0) << "\n";
                    cout << spl.at(1) << "\n";
                    cout << spl.at(2) << "\n";
                    zmq_send(responder, "Received", 5, 0);
                    //If message contains int or "accept" do movement 
                    if (m.askPosition) {
                        int x, y;
                        x = std::stoi(spl.at(0));
                        y = std::stoi(spl.at(1));
                        m.setDestination(Common::CVector3(x, y, 0));
                    }
                    m.authorizeMovement = true;
                }
                else if (m.askPosition) {
                    int x, y;
                    cout << "\n Enter new destination";
                    cout << "\nPos X :";
                    cin >> x;
                    cout << "\nPos Y :";
                    cin >> y;
                    m.setDestination(Common::CVector3(x, y, 0));
                    m.authorizeMovement = true;
                }
                else {
                    cout << "Hit [ENTER] to authorize movement";
                    cin.get();
                    m.authorizeMovement = true;
                }
            }
        }
        cout << "Path Ended";
        m.stop();
    }
    else {
        if (saveMouvement) {
            float minspeed = 0.1;
            float minMouvement = sqrt((3 + 5) * (3 + 5) + (4 + 0) * (4 + 0));
            float updatedspeed = minspeed;
            m.init(posxAudio, posyAudio, 0, true, updatedspeed);
            m.path.clear();
            std::vector<tuple<int, int,int,int>> pathSave;// = new std::vector<tuple<int, int>>();
            m.setSpeed(updatedspeed);
            /*pathSave.push_back({ 21,-12,24,0 });
            pathSave.push_back({ 36,-20,21,-12 });
            pathSave.push_back({ 5,0,8,0 });
            pathSave.push_back({ 0,14,7,12 });
            pathSave.push_back({ 12,-7,21,-12 });
            pathSave.push_back({ 12,-21,21,-12 });
            pathSave.push_back({ 0,-8,0,-14 });
            pathSave.push_back({ 4,7,3,4 });
            pathSave.push_back({ 3,4,4,7 });
            pathSave.push_back({ 4,-2,3,-4 });
            pathSave.push_back({ 0,-24,12,-21 });
            pathSave.push_back({ 0,5,3,4 });
            pathSave.push_back({ 21,-36,36,-20 });
            pathSave.push_back({ 4,-7,7,-12 });
            pathSave.push_back({ 0,-14,0,-8 });
            pathSave.push_back({ 4,2,7,4 });
            pathSave.push_back({ 3,-4,0,-5 });
            pathSave.push_back({ 36,-20,41,0 });
            pathSave.push_back({ 21,12,12,7 });
            pathSave.push_back({ 0,-5,0,-8 });
            pathSave.push_back({ 4,2,5,0 });
            pathSave.push_back({ 21,36,12,21 });
            pathSave.push_back({ 12,7,21,12 });
            pathSave.push_back({ 12,21,21,12 });
            pathSave.push_back({ 7,-12,12,-7 });
            pathSave.push_back({ 21,-12,12,-7 });
            pathSave.push_back({ 14,0,12,-7 });
            pathSave.push_back({ 4,-2,5,0 });
            pathSave.push_back({ 0,24,0,14 });
            pathSave.push_back({ 12,7,14,0 });
            pathSave.push_back({ 3,-4,4,-2 });
            pathSave.push_back({ 5,0,4,-2 });
            pathSave.push_back({ 0,41,0,24 });
            pathSave.push_back({ 12,21,0,24 });
            pathSave.push_back({ 0,-41,0,-24 });
            pathSave.push_back({ 3,4,0,5 });
            pathSave.push_back({ 0,8,0,14 });
            pathSave.push_back({ 7,-4,8,0 });
            pathSave.push_back({ 8,0,7,-4 });
            pathSave.push_back({ 36,20,21,12 });
            pathSave.push_back({ 4,7,7,4 });
            pathSave.push_back({ 41,0,24,0 });
            pathSave.push_back({ 21,36,0,41 });
            pathSave.push_back({ 4,-7,3,-4 });
            pathSave.push_back({ 12,21,21,36 });
            pathSave.push_back({ 7,12,4,7 });
            pathSave.push_back({ 7,-4,12,-7 });
            pathSave.push_back({ 12,7,7,12 });
            pathSave.push_back({ 7,12,12,7 });
            pathSave.push_back({ 4,-7,7,-4 });
            pathSave.push_back({ 24,0,14,0 });
            pathSave.push_back({ 7,12,12,21 });
            pathSave.push_back({ 0,41,21,36 });
            pathSave.push_back({ 0,-8,0,-5 });
            pathSave.push_back({ 7,4,4,7 });
            pathSave.push_back({ 0,8,4,7 });
            pathSave.push_back({ 0,-14,0,-24 });
            pathSave.push_back({ 14,0,8,0 });
            pathSave.push_back({ 12,-21,0,-24 });
            pathSave.push_back({ 12,7,7,4 });
            pathSave.push_back({ 24,0,21,-12 });
            pathSave.push_back({ 21,-36,12,-21 });
            pathSave.push_back({ 0,24,12,21 });
            pathSave.push_back({ 12,-7,7,-4 });
            pathSave.push_back({ 0,-14,7,-12 });
            pathSave.push_back({ 7,4,12,7 });
            pathSave.push_back({ 0,8,0,5 });
            pathSave.push_back({ 0,14,0,24 });
            pathSave.push_back({ 36,20,41,0 });
            pathSave.push_back({ 21,12,36,20 });
            pathSave.push_back({ 7,-12,4,-7 });
            pathSave.push_back({ 0,-24,0,-41 });
            pathSave.push_back({ 7,-4,4,-2 });
            pathSave.push_back({ 36,-20,21,-36 });
            pathSave.push_back({ 36,20,21,36 });
            pathSave.push_back({ 24,0,41,0 });
            pathSave.push_back({ 7,12,0,14 });
            pathSave.push_back({ 12,-7,7,-12 });
            pathSave.push_back({ 12,-21,7,-12 });
            pathSave.push_back({ 21,12,12,21 });
            pathSave.push_back({ 3,4,4,2 });
            pathSave.push_back({ 0,-8,4,-7 });
            pathSave.push_back({ 41,0,36,-20 });
            pathSave.push_back({ 4,-2,7,-4 });
            pathSave.push_back({ 24,0,21,12 });
            pathSave.push_back({ 7,4,4,2 });
            pathSave.push_back({ 0,-5,3,-4 });
            pathSave.push_back({ 8,0,5,0 });
            pathSave.push_back({ 0,5,0,8 });
            pathSave.push_back({ 7,-12,0,-14 });
            pathSave.push_back({ 8,0,14,0 });
            pathSave.push_back({ 0,14,0,8 });*/
            pathSave.push_back({ 4,7,0,8 });
            pathSave.push_back({ 21,12,24,0 });
            pathSave.push_back({ 12,21,7,12 });
            pathSave.push_back({ 41,0,36,20 });
            pathSave.push_back({ 0,-24,0,-14 });
            pathSave.push_back({ 14,0,24,0 });
            pathSave.push_back({ 4,2,3,4 });
            pathSave.push_back({ 21,-12,12,-21 });
            pathSave.push_back({ 14,0,12,7 });
            pathSave.push_back({ 12,-21,21,-36 });
            pathSave.push_back({ 21,36,36,20 });
            pathSave.push_back({ 21,-12,36,-20 });
            pathSave.push_back({ 21,-36,0,-41 });
            pathSave.push_back({ 12,-7,14,0 });
            pathSave.push_back({ 4,-7,0,-8 });
            pathSave.push_back({ 5,0,4,2 });
            pathSave.push_back({ 3,-4,4,-7 });
            pathSave.push_back({ 0,-41,21,-36 });
            pathSave.push_back({ 0,24,0,41 });
            pathSave.push_back({ 7,-4,4,-7 });
            pathSave.push_back({ 4,7,7,12 });
            pathSave.push_back({ 8,0,7,4 });
            pathSave.push_back({ 7,-12,12,-21 });
            pathSave.push_back({ 7,4,8,0 });
            pathSave.push_back({ 0,0,0,0 });
            
            while (pathSave.size() > 0) {
                bool delai = true;
                if (delai) {
                    sleep_until(system_clock::now() + milliseconds(500));
                }
                tuple<int, int, int, int> pos = pathSave.front();
                pathSave.erase(pathSave.begin());
                posxInitAudio = get<0>(pos);
                posyInitAudio = get<1>(pos);
                posxAudio = get<2>(pos);
                posyAudio = get<3>(pos);
                m.setSpeed(1);
                m.setDestination(posxInitAudio, posyInitAudio, 0);
                //m.setPosition(posxAudio, posyAudio);
                while (!(sourcePosition.GetPosition().x == posxInitAudio && sourcePosition.GetPosition().y == posyInitAudio)) {
                    
                    m.updatePosition();
                    sleep_until(system_clock::now() + milliseconds(50));
                }

                bufferSave.clear();
                bufferSave.resize(2);

                float dist = sqrt((posxAudio - posxInitAudio) * (posxAudio - posxInitAudio) + (posyAudio - posyInitAudio) * (posyAudio - posyInitAudio));
                updatedspeed = minspeed * (dist / minMouvement);
                m.setSpeed(0.1*updatedspeed); 
                cout << " debut dest : "<< posxAudio<<"/"<<posyAudio<< "pos"<<posxInitAudio<<"/"<<posyInitAudio<<"\n";
                m.setDestination(posxAudio, posyAudio, 0);
                //m.setPosition(posxAudio, posyAudio);
                while (!(sourcePosition.GetPosition().x == posxAudio && sourcePosition.GetPosition().y == posyAudio)) {
                    float actualdist = sqrt((m.getPosition().x - posxAudio) * (m.getPosition().x - posxAudio) + (m.getPosition().y - posyAudio) * (m.getPosition().y - posyAudio));
                    float s = getSpeedBiologicalMouvement(actualdist / dist)* updatedspeed;
                    //cout << "Distance totale" << dist << " Distance actuelle " << actualdist<<"\n";
                    //cout <<"Distance "<<actualdist/dist<< " Speed "<< s << "\n";
                    //cout << "Calcul " << getSpeedBiologicalMouvement(actualdist / dist) << "\n";
                    if(!isnan(s))
                        m.setSpeed(s);
                    m.updatePosition();
                    sleep_until(system_clock::now() + milliseconds(50));
                }
                if (delai) {
                    sleep_until(system_clock::now() + milliseconds(500));
                }
                AudioFile<double> audioFile;
                string saveFileName = "audioMosquitoFrom" + to_string(posxInitAudio) + "_" + to_string(posyInitAudio) + "to" + to_string(posxAudio) + "_" + to_string(posyAudio) + ".wav";
                audioFile.setAudioBuffer(bufferSave);
                audioFile.save(saveFileName);
                cout << "Save done " << saveFileName<<"\n";
                //cout << " fin pos " << sourcePosition.GetPosition().x << "/" << sourcePosition.GetPosition().y << "\n";
            }
        }
        else {

            m.init(posxAudio, posyAudio, 0, true, 10);
            m.path.clear();
            std::vector<tuple<int, int>> pathSave;// = new std::vector<tuple<int, int>>();
            pathSave.push_back({ 0,1 });
            pathSave.push_back({ 0,-50  });
            /*pathSave.push_back({ 5,0 });
            pathSave.push_back({ 4,2 });
            pathSave.push_back({ 3,4 });
            pathSave.push_back({ 0,5 });
            pathSave.push_back({ 0,-8 });
            pathSave.push_back({ 4,-7 });
            pathSave.push_back({ 7,-4 });
            pathSave.push_back({ 8,0 });
            pathSave.push_back({ 7,4 });
            pathSave.push_back({ 4,7 });
            pathSave.push_back({ 0,8 });
            pathSave.push_back({ 0,-14 });
            pathSave.push_back({ 7,-12 });
            pathSave.push_back({ 12,-7 });
            pathSave.push_back({ 14,0 });
            pathSave.push_back({ 12,7 });
            pathSave.push_back({ 7,12 });
            pathSave.push_back({ 0,14 });
            pathSave.push_back({ 0,-24 });
            pathSave.push_back({ 12,-21 });
            pathSave.push_back({ 21,-12 });
            pathSave.push_back({ 24,0 });
            pathSave.push_back({ 21,12 });
            pathSave.push_back({ 12,21 });
            pathSave.push_back({ 0,24 });
            pathSave.push_back({ 0,-41 });
            pathSave.push_back({ 21,-36 });
            pathSave.push_back({ 36,-20 });
            pathSave.push_back({ 41,0 });
            pathSave.push_back({ 36,20 });
            pathSave.push_back({ 21,36 });
            pathSave.push_back({ 0,41 });*/

            while (pathSave.size() > 0 ) {
                tuple<int, int> pos = pathSave.front();
                pathSave.erase(pathSave.begin());
                posxAudio = get<0>(pos);
                posyAudio = get<1>(pos);
                m.setDestination(posxAudio, posyAudio, 0);
                m.setPosition(posxAudio, posyAudio);
                while (m.getPosition().x != m.destination.x && m.getPosition().y != m.destination.y) {
                    m.updatePosition();
                }

                sleep_until(system_clock::now() + seconds(2));
                bufferSave.clear();

                bufferSave.resize(2);
                bufferSave[0].resize(100000);
                bufferSave[1].resize(100000);
            
                sleep_until(system_clock::now() + seconds(3));
            
                AudioFile<double> audioFile;
                audioFile.setAudioBuffer(bufferSave);
                audioFile.save("audioMosquito" + to_string(posxAudio) + "_" + to_string(posyAudio) + ".wav");
                cout << "Save done " << "audioMosquito" + to_string(posxAudio) + "_" + to_string(posyAudio) + ".wav";
                sleep_until(system_clock::now() + seconds(1));
            }
        }

    }
    if (saveAudio) {
    }
    return 0;
}

int SelectAudioDevice() {
	int connectedAudioDevices = audio->getDeviceCount();
	cout << "     List of available audio outputs" << endl;
	cout << "----------------------------------------" << endl;
	for (int i = 0; i < connectedAudioDevices; i++) {
		cout << "ID: " << i << "-" << audio->getDeviceInfo(i).name << endl;
	}
	int selectAudioDevice;
	//cout << "Please choose which audio output you wish to use: ";
	//cin >> selectAudioDevice; cin.ignore();	
	do {		
		cout << "Please choose which audio output you wish to use: ";
		cin >> selectAudioDevice;
		cin.clear();
		cin.ignore(INT_MAX, '\n');
	} while (!(selectAudioDevice > -1 && selectAudioDevice <= connectedAudioDevices));
	
	return selectAudioDevice;
}

static int rtAudioCallback(void *outputBuffer, void *inputBuffer, unsigned int uiBufferSize, double streamTime, RtAudioStreamStatus status, void *data)
{
    // Setting the output buffer as float
    float * floatOutputBuffer = (float *)outputBuffer;

    // Checking if there is underflow or overflow
    if (status) cout << "stream over/underflow detected";

  	// Initializes buffer with zeros
	  outputBufferStereo.left.Fill(uiBufferSize, 0.0f);
	  outputBufferStereo.right.Fill(uiBufferSize, 0.0f);


    // Getting the processed audio
    audioProcess(outputBufferStereo, uiBufferSize);

    // Declaration and initialization of interlaced audio vector for correct stereo output
    CStereoBuffer<float> iOutput;
    iOutput.Interlace(outputBufferStereo.left, outputBufferStereo.right);

    // Buffer filling loop
    for (auto it = iOutput.begin(); it != iOutput.end(); it++)
    {
        floatOutputBuffer[0] = *it;						 // Setting of value in actual buffer position
        floatOutputBuffer = &floatOutputBuffer[1];				 // Updating pointer to next buffer position
    }

    // Moving the steps source
    /*sourcePosition.SetPosition(Common::CVector3(sourcePosition.GetPosition().x,
                                                sourcePosition.GetPosition().y - streamTime / 110.0f,
                                                sourcePosition.GetPosition().z > 10 ? sourcePosition.GetPosition().z : sourcePosition.GetPosition().z + streamTime / 110.0f));
    */
    sourcePosition.SetPosition(m.getPosition());
    sourceSteps->SetSourceTransform(sourcePosition);
    return 0;
}

void audioProcess(Common::CEarPair<CMonoBuffer<float>> & bufferOutput, int uiBufferSize)
{
    // Declaration, initialization and filling mono buffers
    CMonoBuffer<float> stepsInput (uiBufferSize);	FillBuffer(stepsInput,  wavSamplePositionSteps,  positionEndFrameSteps,  samplesVectorSteps );

    // Declaration of stereo buffer
    Common::CEarPair<CMonoBuffer<float>> bufferProcessed;



    // Anechoic process of steps source
    sourceSteps->SetBuffer(stepsInput);
    sourceSteps->ProcessAnechoic(bufferProcessed.left, bufferProcessed.right);

    // Adding anechoic processed steps source to the output mix
    bufferOutput.left += bufferProcessed.left;
    bufferOutput.right += bufferProcessed.right;

    // Declaration and initialization of separate buffer needed for the reverb
    Common::CEarPair<CMonoBuffer<float>> bufferReverb;

    // Reverberation processing of all sources
    if(false){
           environment->ProcessVirtualAmbisonicReverb(bufferReverb.left, bufferReverb.right);
	    // Adding reverberated sound to the output mix
	    bufferOutput.left += bufferReverb.left;
	    bufferOutput.right += bufferReverb.right;
    }

    AudioFile<double>::AudioBuffer testB;
    for (int i = 0; i < bufferOutput.left.size(); i++) {
        bufferSave[0].push_back(bufferOutput.left.at(i));
        bufferSave[1].push_back(bufferOutput.right.at(i));
    }

}

void FillBuffer(CMonoBuffer<float> &output, unsigned int& position, unsigned int& endFrame, std::vector<float>& samplesVector)
{
    position = endFrame + 1;							 // Set starting point as next sample of the end of last frame
    if (position >= samplesVector.size())				 // If the end of the audio is met, the position variable must return to the beginning
        position = 0;

    endFrame = position + output.size() - 1;			 // Set ending point as starting point plus frame size
    for (int i = 0; i < output.size(); i++) {
        if ((position + i) < samplesVector.size())
            output[i] = (samplesVector[position + i]);	 // Fill with audio
        else
            output[i] = 0.0f;							 // Fill with zeros if the end of the audio is met
    }
}

void LoadWav(std::vector<float>& samplesVector, const char* stringIn)
{
    struct WavHeader								 // Local declaration of wav header struct type (more info in http://soundfile.sapp.org/doc/WaveFormat/)
    {												 // We only need the number of samples, so the rest will be unused assuming file is mono, 16-bit depth and 44.1kHz sampling rate
        char		  fill[40];
        uint32_t	bytesCount;
    } wavHeader;

    FILE* wavFile = fopen(stringIn, "rb");											 // Opening of the wav file
    fread(&wavHeader, sizeof(wavHeader), 1, wavFile);								 // Reading of the 44 bytes of header to get the number of samples of the file
    fseek(wavFile, sizeof(wavHeader), SEEK_SET);									 // Moving of the file pointer to the start of the audio samples

    unsigned int samplesCount = wavHeader.bytesCount / 2;							 // Getting number of samples by dividing number of bytes by 2 because we are reading 16-bit samples
    int16_t *sample; sample = new int16_t[samplesCount];							 // Declaration and initialization of 16-bit signed integer pointer
    memset(sample, 0, sizeof(int16_t) * samplesCount);								 // Setting its size

    uint8_t *byteSample; byteSample = new uint8_t[2 * samplesCount];				 // Declaration and initialization of 8-bit unsigned integer pointer
    memset(byteSample, 0, sizeof(uint8_t) * 2 * samplesCount);						 // Setting its size

    fread(byteSample, 1, 2 * samplesCount, wavFile);								 // Reading the whole file byte per byte, needed for endian-independent wav parsing

    for (int i = 0; i < samplesCount; i++)
        sample[i] = int16_t(byteSample[2 * i] | byteSample[2 * i + 1] << 8);		 // Conversion from two 8-bit unsigned integer to a 16-bit signed integer

    samplesVector.reserve(samplesCount);											 // Reserving memory for samples vector

    for (int i = 0; i < samplesCount; i++)
        samplesVector.push_back((float)sample[i] / (float)INT16_MAX);				 // Converting samples to float to push them in samples vector
}
void AudioLoader(std::vector<float>& samplesVector, const char* stringIn) {
    AudioFile<float> audioFile;
    audioFile.load(stringIn);

    int channel = 0;
    int numSamples = audioFile.getNumSamplesPerChannel();

    for (int i = 0; i < numSamples; i++)
    {
        samplesVector.push_back(audioFile.samples[channel][i]);
    }
}



void initAudio() {
    int iBufferSize;
    bool bEnableReverb;
    iBufferSize = 2048;
    bEnableReverb = true;
    // Core setup
    Common::TAudioStateStruct audioState;	    // Audio State struct declaration
    audioState.bufferSize = iBufferSize;			// Setting buffer size and sample rate
    audioState.sampleRate = SAMPLERATE;       //44100;
    myCore.SetAudioState(audioState);		      // Applying configuration to core
    myCore.SetHRTFResamplingStep(15);		      // Setting 15-degree resampling step for HRTF


    ERRORHANDLER3DTI.SetVerbosityMode(VERBOSITYMODE_ERRORSANDWARNINGS);
    ERRORHANDLER3DTI.SetErrorLogStream(&std::cout, true);


    // Listener setup
    listener = myCore.CreateListener();								 // First step is creating listener
    Common::CTransform listenerPosition = Common::CTransform();		 // Setting listener in (0,0,0)
    listenerPosition.SetPosition(Common::CVector3(0, 0, 0));
    listener->SetListenerTransform(listenerPosition);
    listener->DisableCustomizedITD();								 // Disabling custom head radius

    HRTF::CreateFrom3dti("hrtf.3dti-hrtf", listener);			 // Comment this line and uncomment next lines to load the default HRTF in SOFA format instead of in 3dti-hrtf format

 // Environment setup
    environment = myCore.CreateEnvironment();									// Creating environment to have reverberated sound
    environment->SetReverberationOrder(TReverberationOrder::BIDIMENSIONAL);		// Setting number of ambisonic channels to use in reverberation processing
    BRIR::CreateFromSofa("brir.sofa", environment);								// Loading SOFAcoustics BRIR file and applying it to the environment


    sourceSteps = myCore.CreateSingleSourceDSP();	
    if (equalizedSound) {
        // Creating audio source
        AudioLoader(samplesVectorSteps, "MosquitoFlyEq.wav");
    }else{
        // Creating audio source
        AudioLoader(samplesVectorSteps, "MosquitoFly.wav");
    }
    Common::CTransform sourceStepsPosition = Common::CTransform();
    sourceStepsPosition.SetPosition(Common::CVector3(posxAudio, posyAudio, 0));						 // Setting source in (-3,-10,-10)
    sourceSteps->SetSourceTransform(sourceStepsPosition);
    sourceSteps->SetSpatializationMode(Binaural::TSpatializationMode::HighQuality);		 // Choosing high quality mode for anechoic processing
    sourceSteps->DisableNearFieldEffect();												 // Audio source will not be close to listener, so we don't need near field effect
    sourceSteps->EnableAnechoicProcess();												   // Setting anechoic and reverb processing for this source
    sourceSteps->EnableDistanceAttenuationAnechoic();
    sourcePosition = sourceStepsPosition;												 // Saving initial position into source position to move the steps audio source later on


    // Declaration and initialization of stereo buffer
    outputBufferStereo.left.resize(iBufferSize);
    outputBufferStereo.right.resize(iBufferSize);


    // Audio output configuration, using RtAudio (more info in https://www.music.mcgill.ca/~gary/rtaudio/)
    audio = std::shared_ptr<RtAudio>(new RtAudio());  // Initialization of RtAudio
                                                      // It uses the first API it founds compiled and requires of preprocessor definitions
                                                      // which depends on the OS used and the audio output device (more info in https://www.music.mcgill.ca/~gary/rtaudio/compiling.html)

    // Setting the output parameters
    RtAudio::StreamParameters outputParameters;
    outputParameters.nChannels = 2;									 // Setting output as stereo 

    //outputParameters.deviceId = audio->getDefaultOutputDevice();	 // Choosing default output device
    outputParameters.deviceId = SelectAudioDevice();								// Give user the option to choose the output device	



    // Setting the audio stream options flags.
    RtAudio::StreamOptions options;
    options.flags = RTAUDIO_SCHEDULE_REALTIME;
    options.numberOfBuffers = 4;                // Setting number of buffers used by RtAudio
    options.priority = 1;                       // Setting stream thread priority
    unsigned int frameSize = iBufferSize;       // Declaring and initializing frame size variable because next statement needs it

    // Opening of audio stream
    try {
        audio->openStream(&outputParameters,     // Specified output parameters
            nullptr,			                  // Unspecified input parameters because there will not be input stream
            RTAUDIO_FLOAT32,	              // Output buffer will be 32-bit float
            SAMPLERATE,			                    // Sample rate will be 44.1 kHz
            &frameSize,		                // Frame size will be iBufferSize samples
            &rtAudioCallback,	            // Pointer to the function that will be called every time RtAudio needs the buffer to be filled
            nullptr,			                  // Unused pointer to get feedback
            &options			                  // Stream options (real-time stream, 4 buffers and priority)
        );
    }
    catch (RtAudioError& e) {
        std::cout << "\nERROR:\t" << e.getMessage() << '\n' << std::endl;
        exit(0);
    }

    // Starting the stream
    audio->startStream();
}
void stopAudio() {

    audio->stopStream();
    audio->closeStream();
}
void moveAudioSource(int x, int y) {
    if(!saveAudio)
        sourcePosition.SetPosition(Common::CVector3(x,y,0));
}

float getSpeedBiologicalMouvement(float pourcent) {
    float step = 100 * pourcent;
    float res = 0.0;
    if (step < 50) {
        res = 1 / (1 + exp(-(0.4 * step - 10) / 2));
    }
    else {
        res = 1 - 1 / (1 + exp(-(0.4 * step - 30) / 2));
    }
    if (res < 0.1) {
        res = 0.1;
    }
    return res;
}
void Moustique::init(int posX, int posY, int posZ, bool askPos,float speed) {
    this->position.x = posX;
    this->position.y = posY;
    this->position.z = posZ;
    this->speed = 1;
    this->ended = false;
    this->speed = speed;
    this->authorizeMovement = false;
    this->askPosition = askPos;
    initAudio();
    this->setDefaultPath();
    this->getNextDestination();
}
void Moustique::setSpeed(float newSpeed)
{
    this->speed = newSpeed;
}
void Moustique::stop()
{
    stopAudio();
}

void Moustique::setDefaultPath() {

    this->path.push_back({ 5,-5,0 });
    this->path.push_back({ 0,-15,0 });
    this->path.push_back({ 10,17,0 });
    this->path.push_back({ -4,2,0 });
}
void Moustique::setDestination(int x, int y, int z)
{
    this->destination.x = x;
    this->destination.y = y;
    this->destination.z = z;
}

void Moustique::setPosition(int x, int y) {
    this->position.x = x;
    this->position.y = y;
    this->position.z = 0;
}
void Moustique::setDestination(CVector3 vec)
{
    this->destination.x = vec.x;
    this->destination.y = vec.y;
    this->destination.z = vec.z;
}
CVector3 Moustique::getPosition() {
    this->updatePosition();
    //if(!this->askPosition)
        //cout << "New position " << this->position.x << "/" << this->position.y << "/" << this->position.z<<"\n";
    return this->position;
}
void Moustique::getNextDestination()
{
    CVector3 newDestination;
    if (!this->path.empty()) {
        tuple<int, int, int> newPos = this->path[0];
        newDestination.x = get<0>(newPos);
        newDestination.y = get<1>(newPos);
        newDestination.z = get<2>(newPos);
        this->path.erase(this->path.begin());
        this->destination = newDestination;
        //cout << "New destination : " << newDestination.x <<"/"<< newDestination.y <<"/"<< newDestination.z;
    }
    else {
        cout << "End of path";
        this->ended = true;
    }
}

void Moustique::updatePosition()
{
    CVector3 direction = getDirection();
    //cout << "Destination :" << this->destination.x << "/" << this->destination.y << "/" << this->destination.z << "\n";
    //cout << "Length" << vectorLength(this->destination - this->position) <<"\n";
    if (vectorLength(this->destination - this->position) < 1) {
        position = destination;
        this->waitForAuthorization = true;
        if (this->authorizeMovement) {
            if(!this->askPosition && !saveAudio)
                this->getNextDestination();
            this->authorizeMovement = false;
            this->waitForAuthorization = false;
        }
    }
    else {
        direction.x *= this->speed;
        direction.y *= this->speed;
        direction.z *= this->speed;
        //cout << "Direction :" << direction.x << "/" << direction.y << "/" << direction.z << "\n";
        position = position + direction;
    }
}
CVector3 Moustique::getDirection()
{
    CVector3 direction = this->destination - this->position;
    return normalize(direction);
}
float Moustique::vectorLength(CVector3 vec)
{
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}
CVector3 Moustique::normalize(CVector3 vec)
{
    float length = vectorLength(vec);
    vec.x = vec.x / length;
    vec.y = vec.y / length;
    vec.z = vec.z / length;
    return vec;
}
string convertToString(char* a, int size)
{
    int i;
    string s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}
vector<string> split(const string& str, const string& delim)
{
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos - prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}
/*
extern "C" {
    void Moustique_init() { initialization(false); }
    void Moustique_updatePos(int x, int y) { updatePosition(x,y); }
    void Moustique_stop() { end(); }
}*/