#include <robosub/robosub.h>
#include <robosub/image-processing/parametertuning.h>


namespace robosub {
    map<string, double>
    ParameterTuner::tuneParameters(map<string, double> parametersWithInitialValues, void *evaluationFunction,
                                   TuningMethods method) {
        return map<string, double>();
    }
}
