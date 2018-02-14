#pragma once

#include "common.h"
#include "ws/ws.h"
#include "json/json.hpp"

#include <chrono>
#include <stdexcept>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace robosub {

  typedef nlohmann::json json;

  class DataBucket {
  private:
    json _data;

    long long _getUUID();

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

    //check to ensure that the bucket is inflatable from its previous state
    //each bucket has a unique stamp associated with
    bool isInflatable(DataBucket& previousState);

    //reverse delta compression using previous state
    DataBucket inflate(DataBucket& previousState);

    //remove all data from bucket
    void clear();
  };

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
