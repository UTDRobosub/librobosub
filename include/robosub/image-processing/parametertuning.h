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

        static constexpr double mutationRate = 0.1;
        static constexpr int generationSize = 30;

        double mutateParameter(double currentValue, ParameterMetadata data);

        map<string, double>
        tuneWithGeneticAlgorithm();

        map<string, double> mutateParameters(map<string, double> parameters, double mutRate = mutationRate);

        double generateInitialPopulation(map<string, double> *populationBuffer, vector<double> &populationFitnessLevels,
                                         int populationSize);

        double generateNewPopulation(map<string, double> *populationBuffer, vector<double> &populationFitnessLevels,
                                     int populationSize);

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

        bool saveToFiles(const string &filePrefix, string (*toString)(T));

        bool loadFromFiles(const string &directory, T (*fromString)(string));
    };

    template<typename T>
    class TuningSampleManager {
    public:
        string rootPath;

        TuningSampleManager(const string &rootPath) {
            if (rootPath.at(rootPath.length() - 1) != '/')
                this->rootPath = rootPath + '/';
            else
                this->rootPath = rootPath;
        }

        bool save(TuningSample<T> *samples, int size, string (*toString)(T));

        vector<TuningSample<T>> load(T (*fromString)(string));
    };

    template<typename T>
    bool TuningSampleManager<T>::save(TuningSample<T> *samples, int size, string (*toString)(T)) {
        ofstream sizeFile(rootPath + "sizeFile.dat");
        if (sizeFile.is_open())
            sizeFile << size;
        else
            return false;

        for (int i = 0; i < size; ++i) {
            string directory = rootPath + "Sample" + to_string(i) + "/";
            struct stat buffer;
            if ((stat(directory.c_str(), &buffer) != 0)) {
                mkdir(directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
            }
            samples[i].saveToFiles(directory, toString);
        }
        return true;
    }

    template<typename T>
    bool TuningSample<T>::saveToFiles(const string &directory, string (*toString)(T)) {
        ofstream dataFile(directory + "sampleData.txt");

        if (dataFile.is_open()) {
            for (auto const &data: *sampleData) {
                dataFile << data.first << ":" << toString(data.second) << endl;
            }
        } else {
            return false;
        }

        dataFile.close();

        return imwrite(directory + "image.jpg", image);
    }


    template<typename T>
    bool TuningSample<T>::loadFromFiles(const string &directory, T (*fromString)(string)) {
        ifstream dataFile(directory + "sampleData.txt");

        if (dataFile.is_open()) {
            string line;
            dataFile >> line;
            size_t divider = line.find(':');
            string paramName = line.substr(0, divider);
            T value = fromString(line.substr(divider + 1, line.size() - divider - 1));
            sampleData->insert({paramName, value});
        } else {
            return false;
        }

        dataFile.close();

        image = imread(directory + "image.jpg");
        return image.size != 0;
    }

    template<typename T>
    vector<TuningSample<T>> TuningSampleManager<T>::load(T (*fromString)(string)) {
        ifstream sizeFile(rootPath + "sizeFile.dat");
        int size;
        if (sizeFile.is_open())
            sizeFile >> size;
        else {
            cout << "sizeFile not able to open" << endl;
            return vector<TuningSample<T>>();
        }


        vector<TuningSample<T>> samples(size);

        for (int i = 0; i < size; ++i) {
            string directory = rootPath + "Sample" + to_string(i) + "/";
            samples.at(i).loadFromFiles(directory, fromString);
        }

        return samples;
    }
}


#endif //LIBROBOSUB_PARAMETER_TUNING_GENETIC_ALGORITHM_H
