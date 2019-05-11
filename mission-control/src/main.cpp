
#include <SDL/SDL.h>
#include <iostream>
#include <thread>
#include <librobosub/robosub.h>
#include "main.h"
#include "readout.h"

using namespace std;
//using namespace robosub;

const int NUMFEEDS = 4;
const int PORT[5] = {8500, 8501, 8502, 8503, 8504};
const String VIDEO_ADDR = "192.168.1.2";
const char* NETWORK_HOST = "192.168.1.1:8081";

void control();
void video();
void network(ReadoutData*);

bool running = true;
bool refresh = false;
Controller* controller1;
Controller* controller2;
long controllerTime;
mutex drawLock;

ReadoutData readoutData;

int main(int argc, char* argv[]){
	
	controller1 = new Controller;
	controller2 = new Controller;

	thread controlThread(control);
	thread videoThread(video);
	thread networkThread(network, &readoutData);
//	thread readoutThread(readout, &readoutData);
	
	controlThread.join();
	videoThread.join();
	networkThread.join();
//	readoutThread.join();

	return 0;
}

//Thread for mission control window (everything but video feed)
//Display and send controller inputs,
//Recieve and display diagnostics
void control(){
	
	//start SDL
	if(SDL_Init (SDL_INIT_JOYSTICK | SDL_INIT_VIDEO) < 0 ){
		cout << "Couldn't initialize SDL: " << SDL_GetError() << endl;
		exit(1);
	}
	
	//Uint8 *keystate = SDL_GetKeyState(NULL);
	
	//get controllers
	controller1->setJoystick(SDL_JoystickOpen(0));
	controller2->setJoystick(SDL_JoystickOpen(1));
	
	SDL_Event event;
	//main loop
	while(running){
		
		//event loop
		SDL_PumpEvents();
		while( SDL_PollEvent( &event ) != 0 ) {
			if( event.type == SDL_QUIT )
				running = false;
		}
		
		//refresh when requested or both joysticks are disconnected
		if(refresh || (controller1->mode() == 0) && (controller2->mode() == 0)) {
			refresh = false;
			SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
			SDL_InitSubSystem(SDL_INIT_JOYSTICK);
			controller1->setJoystick(SDL_JoystickOpen(0));
			controller2->setJoystick(SDL_JoystickOpen(1));
		}
		
		controllerTime = robosub::Time::millis(); //add current timestamp
		robosub::Time::waitMillis(1); //prevent pinning the processor at 100%
	}
	
	SDL_Quit();
}
