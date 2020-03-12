#include <robosub/robosub.h>
#include <robosub/image-processing/parametertuning.h>

#include <utility>
#include <algorithm>
#include <iterator>


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
    ParameterTuner::tuneParameters(map<string, ParameterMetadata> parameters,
                                   double (*evalFunction)(map<string, double>),
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
        vector<double> populationErrorLevels(generationSize);

        cout << "Using genetic algorithm for parameter tuning. Generating initial population" << endl;
        initialPopulationAvgError = generateInitialPopulation(populationBuffer, populationErrorLevels, generationSize);
        cout << "Average Error: " << initialPopulationAvgError << endl;

        for (int i = 0; i < 10; ++i) {
            cout << "Starting iteration " << i << " of genetic algorithm training" << endl;
            //put fitness percentage here
            lastAvgError = generateNewPopulation(populationBuffer, populationErrorLevels, generationSize);
            cout << "Average Error: " << lastAvgError << endl;
        }

        cout << "Determining the best possible parameter set" << endl;
        return getBestParameterSet(populationBuffer, generationSize);
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
        bernoulli_distribution bern_distribution = bernoulli_distribution(mutRate);

        for (auto const &kv : parameters) {
            if (bern_distribution(generator)) {
                parameters[kv.first] = mutateParameter(kv.second, parameterMetadata.at(kv.first));
            }
        }

        return parameters;
    }

    double ParameterTuner::generateInitialPopulation(map<string, double> *populationBuffer,
                                                     vector<double> &populationErrorLevels, int populationSize) {
        map<string, double> startingParameters = map<string, double>();
        double totalError = 0;

        for (auto const &kv : parameterMetadata) {
            startingParameters[kv.first] = kv.second.initValue;
        }

        for (int i = 0; i < populationSize; ++i) {
            populationBuffer[i] = mutateParameters(startingParameters);
            populationErrorLevels[i] = evaluationFunction(populationBuffer[i]);
            totalError += populationErrorLevels[i];
        }

        return totalError / populationSize;
    }

    double ParameterTuner::generateNewPopulation(map<string, double> *populationBuffer,
                                                 vector<double> &populationErrorLevels, int populationSize) {
        map<string, double> oldParameters[populationSize];
        copy(populationBuffer, populationBuffer + populationSize, oldParameters);
        double totalError = 0;

        discrete_distribution<int> distribution(populationErrorLevels.begin(), populationErrorLevels.end());

        for (int i = 0; i < populationSize; ++i) {
            // Add index evaluation separately
            auto firstParent = oldParameters[distribution(generator)];
            auto secondParent = oldParameters[distribution(generator)];

            for (auto const &kv: firstParent) {
                bool first = bern_dist(generator);
                if (first) {
                    populationBuffer[i].insert(kv);
                } else {
                    populationBuffer[i].insert({kv.first, secondParent[kv.first]});
                }
            }
        }

        cout << "Getting final parameter values for population:" << endl;
        for (int i = 0; i < populationSize; ++i) {
            populationBuffer[i] = mutateParameters(populationBuffer[i]);

            populationErrorLevels[i] = evaluationFunction(populationBuffer[i]) * difficultyModifier();
            cout << "\tError for parameter set " << i << ": " << populationErrorLevels[i] << endl;
            totalError += populationErrorLevels[i];

        }

        return totalError / populationSize;
    }

    map<string, double> ParameterTuner::getBestParameterSet(map<string, double> *population, int populationSize) {
        tuple<double, map<string, double>> bestParameterSet(-1, map<string, double>());
        for (int i = 0; i < populationSize; ++i) {
            auto error = evaluationFunction(population[i]);
            if (error < get<0>(bestParameterSet) || get<0>(bestParameterSet) == -1)
                bestParameterSet = make_tuple(error, population[i]);
        }

        return get<1>(bestParameterSet);
    }


    ParameterTuner::ParameterTuner() {
        // Create a normal distribution with a good spread for our sigmoid function
        norm_distribution = normal_distribution<double>(0, 5);
        bern_dist = bernoulli_distribution();
    }

    double ParameterTuner::difficultyModifier() {
        auto x = difficultyConstant * (lastAvgError / initialPopulationAvgError);
        return x * x + 1;
    }
}
