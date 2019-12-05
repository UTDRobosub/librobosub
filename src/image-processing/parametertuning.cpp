#include <robosub/robosub.h>
#include <robosub/image-processing/parametertuning.h>

#include <utility>


using namespace std;

namespace robosub {
    ParameterMetadata::ParameterMetadata(double initValue, double minValue, double maxValue) {
        this->initValue = initValue;
        this->minValue = minValue;
        this->maxValue = maxValue;
    }

    double fastSigmoid(double x) {
        return x / (1 + abs(x));
    }

    map<string, double>
    ParameterTuner::tuneParameters(map<string, ParameterMetadata> parameters, void *evalFunction,
                                   TuningMethods method) {
        parameterMetadata = std::move(parameters);
        this->evaluationFunction = evalFunction;
        switch (method) {
            case TuningMethods::GENETIC_ALGORITHM:
                return tuneWithGeneticAlgorithm();
            default:
                return map<string, double>();
        }
    }

    map<string, double>
    ParameterTuner::tuneWithGeneticAlgorithm() {
        map<string, double> populationBuffer[generationSize];
        generateInitialPopulation(populationBuffer, generationSize);

        // TODO: evaluate population fitness and create new population based on fitness of each
    }

    double ParameterTuner::mutateParameter(double currentValue, ParameterMetadata data) {
        double percentChange = norm_distribution(generator);
        percentChange = fastSigmoid(percentChange);

        // If negative percentage, move towards min, otherwise move towards max
        if (percentChange < 0) {
            return currentValue + (currentValue - data.minValue) * percentChange;
        } else {
            return currentValue + (data.maxValue - currentValue) * percentChange;
        }
    }

    map<string, double>
    ParameterTuner::mutateParameters(map<string, double> parameters, double mutRate) {
        bernoulli_distribution bern_distribution = bernoulli_distribution(1 - mutRate);

        for (auto const &kv : parameters) {
            if (bern_distribution(generator)) {
                parameters[kv.first] = mutateParameter(kv.second, parameterMetadata[kv.first]);
            }
        }

        return parameters;
    }

    void ParameterTuner::generateInitialPopulation(map<string, double> *populationBuffer, int populationSize) {
        map<string, double> startingParameters = map<string, double>();
        for (auto const &kv : parameterMetadata) {
            startingParameters[kv.first] = kv.second.initValue;
        }

        for (int i = 0; i < populationSize; ++i) {
            populationBuffer[i] = mutateParameters(parameterMetadata, startingParameters);
        }
    }

    ParameterTuner::ParameterTuner() {
        // Create a normal distribution with a good spread for our sigmoid function
        norm_distribution = normal_distribution<double>(0, 2);
    }
}
