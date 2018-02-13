#include "robosub/primitives.h"

namespace robosub {

     template<>
     int Contour_<int>::type()
     {
         return CV_16SC1;
     }
     template<>
     int Contour_<float>::type()
     {
         return CV_32FC1;
     }
     template<>
     int Contour_<double>::type()
     {
         return CV_64FC1;
     }


}
