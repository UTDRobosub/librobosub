//controller.h
#pragma once

#include <librobosub/robosub.h>
#include <SDL/SDL.h>
#include <string>
#include "robotState.h"

using namespace robosub;

class Controller{
    private:
        SDL_Joystick* joystick = nullptr;

    public:
        void getStates(int*);//should no longer be needed
        void controllerDataBucket(DataBucket&, String);
        void robotDataBucket(DataBucket&, String, int=1);
        int mode();
        void setJoystick(SDL_Joystick*);

};
