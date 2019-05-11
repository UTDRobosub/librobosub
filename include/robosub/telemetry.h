#pragma once

#include "common.h"
#include "timeutil.h"
#include "ws/ws.h"
#include "json/json.hpp"

#include <chrono>
#include <stdexcept>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "sys/times.h"
#include "sys/vtimes.h"
#include "sys/types.h"
#include "sys/sysinfo.h"

namespace robosub {

    typedef nlohmann::json json;

    class DataBucket {
    private:
        json _data;
        bool _compressed = false;

    public:
        using BasicJsonType = nlohmann::basic_json<>;
        using reference =
        typename std::conditional<std::is_const<BasicJsonType>::value,
                typename BasicJsonType::const_reference,
                typename BasicJsonType::reference>::type;

        //initialize an empty data bucket
        DataBucket();

        DataBucket(json data);

        //initialize a bucket from a serialized string
        DataBucket(string string);

        //initialize a bucket from a CBOR binary array
        DataBucket(vector<uint8_t> cborFormat);

        //get or set the value of an object by its key
        reference operator[](const string key);
        //get position in array
        reference operator[](const int key);

        //copy operator
        DataBucket& operator=(const DataBucket& other);

        //is compressed?
        bool isCompressed();

        //get json object
        json toJson();

        //convert to a binary format
        vector<uint8_t> toCbor();

        //convert to string
        string toString();

        //convert to pretty, well-indented string
        string toPrettyString();

        //perform delta compression on the bucket
        DataBucket compress(DataBucket& previousState);

        inline friend ostream& operator<<(ostream & lhs, const DataBucket & rhs) {
            return lhs << rhs._data.dump();
        }

        //reverse delta compression using previous state
        DataBucket inflate(DataBucket& previousState);

        //remove all data from bucket
        void clear();
        
        void remove(const string key);

        bool hasKey(const string key);

        bool isEmpty();
    };

    class Telemetry {
    private:
        unsigned long long _lastTotalUser, _lastTotalUserLow, _lastTotalSys, _lastTotalIdle;
        double _lastCPUPercent = 0.0;
        long long _lastSystemCPUTime = 0;
        long long _lastMemoryPoll = 0;
        struct sysinfo _prevMemInfo;

        void _initCPUCounter();
        struct sysinfo _pollMemory();

    public:
        Telemetry();

        double getSystemCPUUsage();

        unsigned long long getTotalVirtualMemory();
        unsigned long long getTotalVirtualMemoryUsed();

        unsigned long long getTotalPhysicalMemory();
        unsigned long long getTotalPhysicalMemoryUsed();

        double getSystemRAMUsage();
    };

    //WebSocket server that ensures that stores connection states for each client and ensures that the newest compressed datagrams are sent to each client
    // class BucketServer : robosub::ws::SocketServer<robosub::ws::WS> {
    // private:
    //
    // public:
    //   WebsocketServer();
    //
    //
    // };

    // class TelemetryServer {
    // private:
    //   using WsServer = robosub::ws::SocketServer<robosub::ws::WS>;
    //   WsServer server;
    //   void* _onReceive;
    //   bool _enableDeltaCompression;
    //
    // public:
    //
    //   struct Config {
    //   public:
    //     int port;
    //     string address;
    //     bool enableDeltaCompression;
    //   };
    //
    //   TelemetryServer(int port);
    //   ~TelemetryServer();
    //
    //   Endpoint addEndpoint();
    //
    //   void send(json data);
    //
    //   void start();
    //   void close();
    // };
    //
    // class TelemetryClient {
    // private:
    //   using WsClient = robosub::ws::SocketServer<robosub::ws::WS>;
    //   WsClient client;
    //   json _previousData = json::object();
    //   void* _onReceive;
    //
    // public:
    //   TelemetryClient();
    //   ~TelemetryClient();
    //
    //   void send(json data);
    //
    //   void close();
    // };

}
