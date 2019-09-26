#include <iostream>
#include <thread>
#include <robosub/robosub.h>
#include "video-main.h"

using namespace std;
//using namespace robosub;

const int NUMFEEDS = 3;
const int PORT[5] = {8500, 8501, 8502, 8503, 8504};
const String VIDEO_ADDR = "192.168.1.1";
String FILE_PREFIX;

void video();

bool running = true;
bool refresh = false;
mutex drawLock;
VideoWriter videoFiles[NUMFEEDS];

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