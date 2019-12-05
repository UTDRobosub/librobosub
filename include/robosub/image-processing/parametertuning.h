#ifndef LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H
#define LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H

using namespace std;

namespace robosub {
    enum TuningMethods {
        GENETIC_ALGORITHM
    };

    class ParameterTuner {
    private:
        map<string, double> parameters;
        void *evaluationFunction;

    public:
        ParameterTuner() {
        }

        map<string, double> tuneParameters(map<string, double> parametersWithInitialValues, void *evaluationFunction,
                                           TuningMethods method = TuningMethods::GENETIC_ALGORITHM);
    };
}


#endif //LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H
