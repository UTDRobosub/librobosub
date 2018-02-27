#include <robosub/robosub.h>

using namespace robosub;
using WsServer = robosub::ws::SocketServer<robosub::ws::WS>;

#include <stdio.h>
#include <sys/ioctl.h> // For FIONREAD
#include <termios.h>
#include <stdbool.h>
#include <stdlib.h> //rand

int kbhit(void) {
    static bool initflag = false;
    static const int STDIN = 0;

    if (!initflag) {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initflag = true;
    }

    int nbbytes;
    ioctl(STDIN, FIONREAD, &nbbytes);  // 0 is STDIN
    return nbbytes;
}

int main(int argc, char** argv) {
  //prepare buckets to store data
  DataBucket current;
  DataBucket previous;

  //initialize server
  WsServer server;
  server.config.port = 8080;
  server.config.thread_pool_size = 1;
  //server.config.address = "0.0.0.0";

  //endpoint state storage
  std::map<shared_ptr<WsServer::Connection>, DataBucket> connectionState;

  //GET "/": root endpoint
  auto &root = server.endpoint["^/?$"];
  //GET "/": on open
  //When opened, server should send initial state message
  root.on_open = [&connectionState,&current](shared_ptr<WsServer::Connection> connection) {
    cout << "Server: Opened connection " << connection.get() << endl;
    //put current state into state storage
    connectionState[connection] = current;
    //convert current state to stream
    auto send_stream = make_shared<WsServer::SendStream>();
    *send_stream << connectionState[connection];
    //send current state
    connection->send(send_stream, [](const robosub::ws::error_code &ec) {
      if(ec) {
        cout << "Server: Error sending message. " <<
            // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
            "Error: " << ec << ", error message: " << ec.message() << endl;
      }
    });
  };
  //GET "/": on message received
  //TODO send options information to another server
  root.on_message = [&connectionState](shared_ptr<WsServer::Connection> connection, shared_ptr<WsServer::Message> message) {
    //Display message received
    auto message_str = message->string();
    cout << "Server: Message received: " << message_str;
  };
  //GET "/": on close
  root.on_close = [&connectionState](shared_ptr<WsServer::Connection> connection, int status, const string & /*reason*/) {
    connectionState.erase(connection);
    // See RFC 6455 7.4.1. for status codes
    cout << "Server: Closed connection " << connection.get() << " with status code " << status << endl;
  };
  //GET "/": on error
  root.on_error = [&connectionState](shared_ptr<WsServer::Connection> connection, const robosub::ws::error_code &ec) {
    // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
    cout << "Server: Error in connection " << connection.get() << ". "
         << "Error: " << ec << ", error message: " << ec.message() << endl;
  };

  thread server_thread([&server]() {
    // Start WS-server
    server.start();
  });

  //wait one second for server to start
  cout << "Starting server..." << endl;
  robosub::Time::waitMillis(1000);

  int i=0;
  while(true) {
    previous = current;

    robosub::Time::waitMillis(100);

    unsigned long milliseconds_since_epoch =
    std::chrono::system_clock::now().time_since_epoch() /
    std::chrono::milliseconds(1);

    current["rand"] = rand() % 100;
    current["index"] = i++ % 1000;
    current["time"] = milliseconds_since_epoch;

    cout << current << endl;

    //send update to all active connections
    for(auto &connection : server.get_connections())
    {
      //compress data
      DataBucket previousState = connectionState[connection];
      DataBucket compressed = current.compress(previousState);

      //skip if no changes
      if (compressed.toJson().empty()) continue;

      //set new connection state (assume received)
      connectionState[connection] = current;

      //check if better to send as compressed or uncompressed
      if (current.toString().length() < compressed.toString().length())
      {
        //better to send uncompressed
        auto send_stream = make_shared<WsServer::SendStream>();
        *send_stream << current;
        connection->send(send_stream);
      } else {
        //send compressed
        auto send_stream = make_shared<WsServer::SendStream>();
        *send_stream << compressed;
        connection->send(send_stream);
      }

    }
  }

  return 0;
}
