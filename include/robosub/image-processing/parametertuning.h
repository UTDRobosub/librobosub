#ifndef LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H
#define LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H

using namespace std;

namespace robosub {
    enum TuningMethods {
        GENETIC_ALGORITHM
    };

    class ParameterMetadata {
    public:
        double initValue;
        double minValue;
        double maxValue;

        ParameterMetadata(double initValue, double minValue, double maxValue);
    };

    class ParameterTuner {
    private:
        map<string, ParameterMetadata> parameterMetadata;
        void *evaluationFunction;

        default_random_engine generator;
        normal_distribution<double> norm_distribution;

        static constexpr double mutationRate = 0.01;
        static constexpr int generationSize = 100;

        double mutateParameter(double currentValue, ParameterMetadata data);

        map<string, double>
        tuneWithGeneticAlgorithm();

        map<string, double> mutateParameters(map<string, double> parameters, double mutRate = mutationRate);

        void generateInitialPopulation(map<string, double> *populationBuffer, int populationSize = generationSize);

    public:
        ParameterTuner();

        map<string, double>
        tuneParameters(map<string, ParameterMetadata> parameters, void *evalFunction,
                       TuningMethods method = TuningMethods::GENETIC_ALGORITHM);
    };
}


#endif //LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H
