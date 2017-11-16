
#include "robosub/networkimage.h"
/*
template <class type, int vals>
void MatToRaw(Mat *m, int rows, int cols, char *raw){
    for(int y=0; y<rows; y++){
        for(int x=0; x<cols; x++){
            Vec<type,vals> vec = m->at<Vec<type,vals>>(y,x);

            for(int z=0; z<vals; z++){
                matrawval<type> val;
                val.v = vec[z];

                int loc = z + sizeof(type)*(x + y*cols);
                for(int i=0; i<sizeof(type); i++){
                    raw[loc] = val.bytes[i];
                }
            }
        }
    }
}

template <class type, int vals>
void RawToMat(Mat *m, int rows, int cols, char *raw){
    for(int y=0; y<rows; y++){
        for(int x=0; x<cols; x++){
            Vec<type,vals>  &vec = m->at<Vec<type,vals> >(y,x);

            for(int z=0; z<vals; z++){
                matrawval<type> val;

                int loc = z + sizeof(type)*(x + y*cols);
                for(int i=0; i<sizeof(type); i++){
                    val.bytes[i] = raw[loc];
                }

                vec[z] = val.v;
            }
        }
    }
}

#define MATRAWTYPEVALS(type,vals) \
template void RawToMat<type,vals>(Mat*,int,int,char*); \
template void MatToRaw<type,vals>(Mat*,int,int,char*);

#define MATRAWTYPE(type) \
MATRAWTYPEVALS(type,1) \
MATRAWTYPEVALS(type,3)

//add types here to compile template definitions for them
MATRAWTYPE(uchar)
MATRAWTYPE(int)
MATRAWTYPE(uint)
MATRAWTYPE(ushort)
*/
