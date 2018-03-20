#include <robosub/robosub.h>

using namespace std;

using WsServer = robosub::ws::SocketServer<robosub::ws::WS>;
using WsClient = robosub::ws::SocketClient<robosub::ws::WS>;

int main() {
  // WebSocket (WS)-server at port 8080 using 1 thread
  WsServer server;
  server.config.port = 8081;

  auto &echo = server.endpoint["^/echo/?$"];

  echo.on_message = [](shared_ptr<WsServer::Connection> connection, shared_ptr<WsServer::Message> message) {
    auto message_str = message->string();

    cout << "Server: Message received: \"" << message_str << "\" from " << connection.get() << endl;

    //TODO handle data from mission control

    // cout << "Server: Sending message \"" << message_str << "\" to " << connection.get() << endl;
    //
    // auto send_stream = make_shared<WsServer::SendStream>();
    // *send_stream << message_str;
    // // connection->send is an asynchronous function
    // connection->send(send_stream, [](const robosub::ws::error_code &ec) {
    //   if(ec) {
    //     cout << "Server: Error sending message. " <<
    //         // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
    //         "Error: " << ec << ", error message: " << ec.message() << endl;
    //   }
    // });
  };

  echo.on_open = [](shared_ptr<WsServer::Connection> connection) {
    cout << "Server: Opened connection " << connection.get() << endl;
  };

  // See RFC 6455 7.4.1. for status codes
  echo.on_close = [](shared_ptr<WsServer::Connection> connection, int status, const string & /*reason*/) {
    cout << "Server: Closed connection " << connection.get() << " with status code " << status << endl;
  };

  // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
  echo.on_error = [](shared_ptr<WsServer::Connection> connection, const robosub::ws::error_code &ec) {
    cout << "Server: Error in connection " << connection.get() << ". "
         << "Error: " << ec << ", error message: " << ec.message() << endl;
  };

  thread server_thread([&server]() {
    // Start WS-server
    server.start();
  });

  // Wait for server to start so that the client can connect
  this_thread::sleep_for(chrono::seconds(1));

  cout << "Started robot service demo" << endl;

  server_thread.join();
}
