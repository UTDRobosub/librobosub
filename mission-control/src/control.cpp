//
// Created by robosub on 10/2/19.
//

#include <SDL/SDL.h>
#include <iostream>
#include "main.h"

void control(ThreadData* threaddata) {

    //start SDL
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_VIDEO) < 0) {
        cout << "Couldn't initialize SDL: " << SDL_GetError() << endl;
        exit(1);
    }


    //get controllers
    threaddata->controller1->setJoystick(SDL_JoystickOpen(0));
    threaddata->controller2->setJoystick(SDL_JoystickOpen(1));

    SDL_Event event;
    //main loop
    while (running) {

        //event loop
        SDL_PumpEvents();
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT)
                running = false;
        }

        //refresh when requested or both joysticks are disconnected
        if (refresh || ((threaddata->controller1->mode() == 0) && (threaddata->controller2->mode() == 0))) {
            refresh = false;
            SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
            SDL_InitSubSystem(SDL_INIT_JOYSTICK);
            threaddata->controller1->setJoystick(SDL_JoystickOpen(0));
            threaddata->controller2->setJoystick(SDL_JoystickOpen(1));
        }

        controllerTime = robosub::Time::millis(); //add current timestamp
        robosub::Time::waitMillis(1); //prevent pinning the processor at 100%
    }

    SDL_Quit();
}
