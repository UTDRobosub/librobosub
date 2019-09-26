#include "robosub/telemetry.h"

namespace robosub {

    DataBucket::DataBucket() {
        _data = json({});
    }

    DataBucket::DataBucket(json data) {
        _data = data;
    }

    DataBucket::DataBucket(string string) {
        _data = json::parse(string);
    }

    DataBucket::DataBucket(vector <uint8_t> cborFormat) {
        _data = json::from_cbor(cborFormat);
    }

    bool DataBucket::isCompressed() {
        return _compressed;
    }

    DataBucket::reference DataBucket::operator[](const string key) {
        if (isCompressed()) {
            throw runtime_error("Accessors not allowed on compressed buckets");
        }
        return _data[key];
    }

    DataBucket::reference DataBucket::operator[](const int key) {
        if (isCompressed()) {
            throw runtime_error("Accessors not allowed on compressed buckets");
        }
        return _data[key];
    }

    bool DataBucket::isEmpty() {
        return _data.size() == 0;
    }

    bool DataBucket::hasKey(const string key) {

    }

    DataBucket &DataBucket::operator=(const DataBucket &other) {
        _data = other._data;
        return *this;
    }

    json DataBucket::toJson() {
        return _data;
    }

    string DataBucket::toString() {
        return _data.dump();
    }

    string DataBucket::toPrettyString() {
        return _data.dump(4); //number of spaces to indent
    }

    vector <uint8_t> DataBucket::toCbor() {
        return json::to_cbor(_data);
    }

    void DataBucket::remove(const string key) {
        _data.erase(key);
    }

    void DataBucket::clear() {
        _data.clear();
    }

    DataBucket DataBucket::compress(DataBucket &previousState) {
        json diffs = json::diff(previousState._data, _data);

        DataBucket newBucket;
        newBucket._data = diffs;
        newBucket._compressed = true;
        return newBucket;
    }

    DataBucket DataBucket::inflate(DataBucket &previousState) {
        cout << previousState._data << endl;
        cout << _data << endl;

        if (!isCompressed())
            throw runtime_error("Cannot inflate uncompressed data");

        json result = previousState._data.patch(_data);

        DataBucket newBucket;
        newBucket._data = result;
        return newBucket;
    }

    Telemetry::Telemetry() {
        _initCPUCounter();
    }

    void Telemetry::_initCPUCounter() {
        FILE *file = fopen("/proc/stat", "r");
        fscanf(file, "cpu %llu %llu %llu %llu", &_lastTotalUser, &_lastTotalUserLow,
               &_lastTotalSys, &_lastTotalIdle);
        fclose(file);
    }

    double Telemetry::getSystemCPUUsage() {

        if (robosub::Time::millis() < 1000 + _lastSystemCPUTime) return _lastCPUPercent;

        double percent;
        FILE *file;
        unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

        file = fopen("/proc/stat", "r");
        fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow,
               &totalSys, &totalIdle);
        fclose(file);

        if (totalUser < _lastTotalUser || totalUserLow < _lastTotalUserLow ||
            totalSys < _lastTotalSys || totalIdle < _lastTotalIdle) {
            //Overflow detection. Just skip this value.
            return _lastCPUPercent;
        } else {
            total = (totalUser - _lastTotalUser) + (totalUserLow - _lastTotalUserLow) +
                    (totalSys - _lastTotalSys);
            percent = total;
            total += (totalIdle - _lastTotalIdle);
            percent /= total;
            percent *= 100.0;
        }

        _lastTotalUser = totalUser;
        _lastTotalUserLow = totalUserLow;
        _lastTotalSys = totalSys;
        _lastTotalIdle = totalIdle;
        _lastCPUPercent = percent;
        _lastSystemCPUTime = robosub::Time::millis();

        return percent;
    }

    unsigned long long Telemetry::getTotalVirtualMemory() {
        struct sysinfo memInfo = _pollMemory();
        unsigned long long totalVirtualMem = memInfo.totalram;
        //Add other values in next statement to avoid int overflow on right hand side...
        totalVirtualMem += memInfo.totalswap;
        totalVirtualMem *= memInfo.mem_unit;
        return totalVirtualMem;
    }

    unsigned long long Telemetry::getTotalVirtualMemoryUsed() {
        struct sysinfo memInfo = _pollMemory();
        unsigned long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
        //Add other values in next statement to avoid int overflow on right hand side...
        virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
        virtualMemUsed *= memInfo.mem_unit;
        return virtualMemUsed;
    }

    unsigned long long Telemetry::getTotalPhysicalMemory() {
        struct sysinfo memInfo = _pollMemory();
        unsigned long long totalPhysMem = memInfo.totalram;
        //Multiply in next statement to avoid int overflow on right hand side...
        totalPhysMem *= memInfo.mem_unit;
        return totalPhysMem;
    }

    unsigned long long Telemetry::getTotalPhysicalMemoryUsed() {
        struct sysinfo memInfo = _pollMemory();
        unsigned long long physMemUsed = memInfo.totalram - memInfo.freeram;
        //Multiply in next statement to avoid int overflow on right hand side...
        physMemUsed *= memInfo.mem_unit;
        return physMemUsed;
    }

    double Telemetry::getSystemRAMUsage() {
        return (double) getTotalPhysicalMemoryUsed() * 100.0 / (double) getTotalPhysicalMemory();
    }

    struct sysinfo Telemetry::_pollMemory() {
        if (robosub::Time::millis() < 1000 + _lastMemoryPoll) return _prevMemInfo;

        struct sysinfo memInfo;
        sysinfo(&memInfo);
        _prevMemInfo = memInfo;

        return memInfo;
    }

}
