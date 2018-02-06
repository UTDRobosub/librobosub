
#include "robosub/networkvideo.h"

namespace robosub{
    void SendFrame(UDPS *udps, Mat *frame){
        int rows = frame->rows;
        int cols = frame->cols;


    }

    Mat *RecvFrame(UDPR *udpr){
        return 0;
    }
}
