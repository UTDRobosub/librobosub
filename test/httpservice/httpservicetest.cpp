#include <robosub/robosub.h>
#include <stdio.h>
#include <stdlib.h> //rand

using namespace std;
using namespace robosub;

using WsServer = robosub::ws::SocketServer<robosub::ws::WS>;
using WsClient = robosub::ws::SocketClient<robosub::ws::WS>;

struct ConnectionState {
public:
    bool ready = false;
    long lastSend = 0;
    long rtt = 0;
};

bool handleSpecialMissionControlMessage(string message) {
    return false; //nothing to handle
}

void handleMissionControlState(DataBucket state) {

}

int main(int argc, char** argv) {
    //prepare buckets to store data
    DataBucket current;
    DataBucket receivedState;

    //initialize server
    WsServer server;
    server.config.port = 8081;
    server.config.thread_pool_size = 1;
    server.config.address = "0.0.0.0";

    //endpoint state storage
    std::map<shared_ptr<WsServer::Connection>, DataBucket> connectionData;
    std::map<shared_ptr<WsServer::Connection>, ConnectionState> connectionState;

    auto &root = server.endpoint["^/?$"];

    root.on_open = [&connectionData, &connectionState, &current](shared_ptr<WsServer::Connection> connection) {
        cout << "Server: Opened connection " << connection.get() << endl;

        //put current state into state storage
        connectionData[connection] = current;
        connectionState[connection] = ConnectionState();
        connectionState[connection].lastSend = robosub::Time::millis();

        //convert current state to stream
        auto send_stream = make_shared<WsServer::SendStream>();
        *send_stream << connectionData[connection];

        //send current state
        connection->send(send_stream, [](const robosub::ws::error_code &ec) {
            if(ec) {
                cout << "Server: Error sending message. " <<
                     // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
                     "Error: " << ec << ", error message: " << ec.message() << endl;
            }
        });
    };

    root.on_message = [&connectionState,&receivedState](shared_ptr<WsServer::Connection> connection, shared_ptr<WsServer::Message> message) {
        auto message_str = message->string();

        if (message_str == "\x06") {
            connectionState[connection].ready = true;
            connectionState[connection].rtt = robosub::Time::millis() - connectionState[connection].lastSend;
        } else {
            cout << "Server: Message received: " << message_str << endl;

            //handle messages from mission control
            if (!handleSpecialMissionControlMessage(message_str)) {

                char firstCharacter = message_str.at(0);
                if (firstCharacter == '[') {
                    //data is compressed - inflate
                    DataBucket received = DataBucket(message_str);
                    receivedState = received.inflate(receivedState);
                } else {
                    //data bucket is not compressed
                    receivedState = DataBucket(message_str);
                }

                handleMissionControlState(receivedState);
            }

            //send ACK
            auto send_stream = make_shared<WsServer::SendStream>();
            *send_stream << "\x06"; //ACK
            connection->send(send_stream, [](const robosub::ws::error_code &ec) {
                if(ec) {
                    cout << "Server: Error sending message. " <<
                         // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
                         "Error: " << ec << ", error message: " << ec.message() << endl;
                } else {
                    cout << "Server: Send ACK" << endl;
                }
            });
        }
    };

    // See RFC 6455 7.4.1. for status codes
    root.on_close = [&connectionState, &connectionData](shared_ptr<WsServer::Connection> connection, int status, const string & /*reason*/) {
        connectionState.erase(connection);
        connectionData.erase(connection);
        cout << "Server: Closed connection " << connection.get() << " with status code " << status << endl;
    };

    // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
    root.on_error = [&connectionState, &connectionData](shared_ptr<WsServer::Connection> connection, const robosub::ws::error_code &ec) {
        connectionState.erase(connection);
        connectionData.erase(connection);
        cout << "Server: Error in connection " << connection.get() << ". "
             << "Error: " << ec << ", error message: " << ec.message() << endl;
    };

    thread server_thread([&server]() {
        // Start WS-server
        server.start();
    });

    // Wait for server to start so that the client can connect
    robosub::Time::waitMillis(1000);
    cout << "Server started" << endl;

    int i = 0;

    while(true) {
        current["index"] = i++ % 1000; //force refresh approx every second
        current["rand"] = rand() % 100;

        robosub::Time::waitMillis(1);
        unsigned long milliseconds_since_epoch = robosub::Time::millis();

        //send data to connections
        for(auto &connection : server.get_connections())
        {
            if (!connectionState[connection].ready) continue;

            //compress data
            DataBucket previousState = connectionData[connection];
            DataBucket compressed = current.compress(previousState);

            //skip if no changes
            if (compressed.toJson().empty()) continue;

            //update time-dependent values and recompress
            DataBucket sentState = current;
            sentState["time"] = milliseconds_since_epoch;
            sentState["rtt"] = connectionState[connection].rtt;
            compressed = sentState.compress(previousState);

            //set new connection state
            connectionData[connection] = current;

            //update connection state
            connectionState[connection].ready = false;
            connectionState[connection].lastSend = robosub::Time::millis();

            //check if better to send as compressed or uncompressed
            //cout << current.toString().length() << " " << compressed.toString().length() << endl;
            if (current.toString().length() < compressed.toString().length())
            {
                //better to send uncompressed
                auto send_stream = make_shared<WsServer::SendStream>();
                *send_stream << sentState;
                connection->send(send_stream);
            } else {
                //send compressed
                auto send_stream = make_shared<WsServer::SendStream>();
                *send_stream << compressed;
                connection->send(send_stream);
            }

        }
    }

    server_thread.join();
}
