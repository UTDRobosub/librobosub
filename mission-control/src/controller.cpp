//controller.cpp

#include <librobosub/robosub.h>
#include <SDL/SDL.h>
#include "controller.h"
#include "robotState.h"
using namespace std;
using namespace robosub;


void Controller::setJoystick(SDL_Joystick* j){
    joystick = j;
}

int Controller::mode(){
    if(joystick == nullptr)
        return 0;
    if(SDL_JoystickNumAxes(joystick) == 4)      //D
        return 1;
    if(SDL_JoystickNumAxes(joystick) == 6)      //X
        return 2;
}

void Controller::controllerDataBucket(DataBucket &b, String s){

    b[s] = {};

    switch(mode()) {
        case 1:
            b[s]["connected"] = 1;
            b[s]["mode"] = "D";
            b[s]["lx"] = SDL_JoystickGetAxis(joystick, 0) / 128.0 / 256.0;
            b[s]["ly"] = SDL_JoystickGetAxis(joystick, 1) / 128.0 / 256.0;
            b[s]["rx"] = SDL_JoystickGetAxis(joystick, 2) / 128.0 / 256.0;
            b[s]["ry"] = SDL_JoystickGetAxis(joystick, 3) / 128.0 / 256.0;
            b[s]["a"] = SDL_JoystickGetButton(joystick, 1);
            b[s]["b"] = SDL_JoystickGetButton(joystick, 2);
            b[s]["x"] = SDL_JoystickGetButton(joystick, 0);
            b[s]["y"] = SDL_JoystickGetButton(joystick, 3);
            b[s]["lb"] = SDL_JoystickGetButton(joystick, 4);
            b[s]["rb"] = SDL_JoystickGetButton(joystick, 5);
            b[s]["select"] = SDL_JoystickGetButton(joystick, 8);
            b[s]["start"] = SDL_JoystickGetButton(joystick, 9);
            b[s]["ldown"] = SDL_JoystickGetButton(joystick, 10);
            b[s]["rdown"] = SDL_JoystickGetButton(joystick, 11);
            b[s]["lt"] = SDL_JoystickGetButton(joystick, 6);
            b[s]["rt"] = SDL_JoystickGetButton(joystick, 7);
            b[s]["hat"] = SDL_JoystickGetHat(joystick, 0);
            if (b[s]["hat"] == 1 || b[s]["hat"] == 3 || b[s]["hat"] == 9)
                b[s]["up"] = 1;
            else
                b[s]["up"] = 0;
            if (b[s]["hat"] == 2 || b[s]["hat"] == 3 || b[s]["hat"] == 6)
                b[s]["right"] = 1;
            else
                b[s]["right"] = 0;
            if (b[s]["hat"] == 4 || b[s]["hat"] == 6 || b[s]["hat"] == 12)
                b[s]["down"] = 1;
            else
                b[s]["down"] = 0;
            if (b[s]["hat"] == 8 || b[s]["hat"] == 12 || b[s]["hat"] == 9)
                b[s]["left"] = 1;
            else
                b[s]["left"] = 0;
            break;
        case 2:
            b[s]["connected"] = 1;
            b[s]["mode"] = "X";
            b[s]["lx"] = SDL_JoystickGetAxis(joystick, 0) / 128.0 / 256.0;
            b[s]["ly"] = SDL_JoystickGetAxis(joystick, 1) / 128.0 / 256.0;
            b[s]["rx"] = SDL_JoystickGetAxis(joystick, 3) / 128.0 / 256.0;
            b[s]["ry"] = (short) (SDL_JoystickGetAxis(joystick, 4) / 128.0); //this one's for verical motors, [-255,255]
            b[s]["a"] = SDL_JoystickGetButton(joystick, 0);
            b[s]["b"] = SDL_JoystickGetButton(joystick, 1);
            b[s]["x"] = SDL_JoystickGetButton(joystick, 2);
            b[s]["y"] = SDL_JoystickGetButton(joystick, 3);
            b[s]["lb"] = SDL_JoystickGetButton(joystick, 4);
            b[s]["rb"] = SDL_JoystickGetButton(joystick, 5);
            b[s]["select"] = SDL_JoystickGetButton(joystick, 6);
            b[s]["start"] = SDL_JoystickGetButton(joystick, 7);
            b[s]["ldown"] = SDL_JoystickGetButton(joystick, 9);
            b[s]["rdown"] = SDL_JoystickGetButton(joystick, 10);

            b[s]["t"] = (SDL_JoystickGetAxis(joystick, 5) - SDL_JoystickGetAxis(joystick, 2)) / 128.0 / 512.0;
            if (b[s]["t"] > 2) {
                b[s]["lt"] = 1;
                b[s]["rt"] = 0;
            } else if (b[s]["t"] < -2) {
                b[s]["lt"] = 0;
                b[s]["rt"] = 1;
            } else {
                b[s]["lt"] = 0;
                b[s]["rt"] = 0;
            }
            b[s]["hat"] = SDL_JoystickGetHat(joystick, 0);
            if (b[s]["hat"] == 1 || b[s]["hat"] == 3 || b[s]["hat"] == 9)
                b[s]["up"] = 1;
            else
                b[s]["up"] = 0;
            if (b[s]["hat"] == 2 || b[s]["hat"] == 3 || b[s]["hat"] == 6)
                b[s]["right"] = 1;
            else
                b[s]["right"] = 0;
            if (b[s]["hat"] == 4 || b[s]["hat"] == 6 || b[s]["hat"] == 12)
                b[s]["down"] = 1;
            else
                b[s]["down"] = 0;
            if (b[s]["hat"] == 8 || b[s]["hat"] == 12 || b[s]["hat"] == 9)
                b[s]["left"] = 1;
            else
                b[s]["left"] = 0;
            break;
        default:
            b[s]["connected"] = 0;
            b[s]["mode"] = "";
            return;
    }
}

