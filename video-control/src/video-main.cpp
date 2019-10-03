#include <iostream>
#include <thread>
#include <robosub/robosub.h>
#include "video-main.h"

using namespace std;
//using namespace robosub;

void video();

bool running = true;
bool refresh = false;
mutex drawLock;
VideoWriter videoFiles[numFeeds];

int main(int argc, char *argv[]) {

    const String keys =
            "{help ?   |                    | print this message     }"
            "{p        | /media/v-patel/uSB | video file prefix      }";

    CommandLineParser parser(argc, argv, keys);
    parser.about("Robosub Video Control");

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    FILE_PREFIX = parser.get<String>("p") + "/";
    cout << FILE_PREFIX << endl;
    video();

    return 0;
}