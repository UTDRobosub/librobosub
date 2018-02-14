#pragma once

#include "common.h"
#include "ws/ws.h"
#include "json/json.hpp"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace robosub {

  typedef nlohmann::json json;

  class DataBucket {
  private:
    json _data = json({ });
    boost::uuids::uuid _uuid;
    boost::uuids::random_generator _uuidgen;

  public:
    using BasicJsonType = nlohmann::basic_json<>;
    using reference =
      typename std::conditional<std::is_const<BasicJsonType>::value,
      typename BasicJsonType::const_reference,
      typename BasicJsonType::reference>::type;

    //initialize an empty data bucket
    DataBucket();

    //initialize a bucket from a CBOR binary array
    DataBucket(std::vector<std::uint8_t> cborFormat);

    //get or set the value of an object by its key
    reference operator[](const string key);

    //get json object
    json toJson();

    //convert to a binary format
    std::vector<std::uint8_t> toCbor();

    //convert to string
    string toString();

    //convert to pretty, well-indented string
    string toPrettyString();

    //perform delta compression on the bucket
    DataBucket& compress(DataBucket previousState);

    //check to ensure that the bucket is inflatable from its previous state
    //each bucket has a unique stamp associated with
    bool isInflatable(DataBucket previousState);

    //reverse delta compression using previous state
    DataBucket& inflate(DataBucket previousState);

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
