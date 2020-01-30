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

        double (*evaluationFunction)(map<string, double>);

        default_random_engine generator;
        normal_distribution<double> norm_distribution;
        bernoulli_distribution bern_dist;

        static constexpr double mutationRate = 0.01;
        static constexpr int generationSize = 30;

        double mutateParameter(double currentValue, ParameterMetadata data);

        map<string, double>
        tuneWithGeneticAlgorithm();

        map<string, double> mutateParameters(map<string, double> parameters, double mutRate = mutationRate);

        void generateInitialPopulation(map<string, double> *populationBuffer, int populationSize = generationSize);

        void generateNewPopulation(map<string, double> *populationBuffer, int populationSize);

        map<string, double> getBestParameterSet(map<string, double> *population, int populationSize);

    public:
        ParameterTuner();

        map<string, double>
        tuneParameters(map<string, ParameterMetadata> parameters, double (*evalFunction)(map<string, double>),
                       TuningMethods method = TuningMethods::GENETIC_ALGORITHM);
    };

    template<typename T>
    class TuningSample {
    public:
        Mat image;
        map<string, T> *sampleData;

        TuningSample() {
            sampleData = new map<string, T>();
        }

        bool saveToFile(string filePrefix, string (*toString)(T));
    };
}


#endif //LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H
