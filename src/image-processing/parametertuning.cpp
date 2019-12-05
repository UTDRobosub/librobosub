#include <robosub/robosub.h>

using namespace std;

namespace robosub {
    map<string, double>
    ParameterTuner::tuneParameters(map<string, double> parametersWithInitialValues, void *evaluationFunction,
                                   TuningMethods method) {
        return map<string, double>();
    }
}
