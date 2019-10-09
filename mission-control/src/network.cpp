
#include <robosub/robosub.h>
#include "robotState.h"
#include "main.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

using namespace robosub;
using WsServer = robosub::ws::SocketServer<robosub::ws::WS>;
using WsClient = robosub::ws::SocketClient<robosub::ws::WS>;

RobotState robotState;

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h> //rand
#include "main.h"

/*
 *
 */
struct ConnectionState {
public:
    bool ready = false;
    long lastSend = 0;
    long rtt = 0;
};

const double MOTOR_POWER = 100.0;
const double MOTOR_DEADBAND = 50.0; //minimum motor power

/**
 * converts given power value (-1 to 1) to power value for robot
 * @param value - double, position of the joystick from -1 to 1
 * @return power - int, power level to run the motor at
 */
int convertMotorValuesToRobot(double value) {
    double power = value * MOTOR_POWER;
    if (abs(power) < MOTOR_DEADBAND) power = 0;

    return (int) (power + 1500.0);
}

int convertVerticalMotorValuesToRobot(double value) {
    return (int) (value / 256.0 * 255.0);
}

int convertArmValuesToRobot(double input, double maxValue) {
    return (int) ((input * maxValue + maxValue) / 2.0);
}

//-1 to 1 => min to max
int convertArmRotationValuesToRobot(double input, double minValue, double maxValue) {
    return (int) ((input * (maxValue - minValue)) + ((maxValue + minValue) / 2.0));
}

template<class T>
void send(shared_ptr<typename T::Connection> connection,
          ConnectionState &connectionState,
          DataBucket &connectionData,
          DataBucket &current,
          unsigned long milliseconds_since_epoch,
          bool compress
) {
    if (!connectionState.ready) return;

    try {
        //compress data
        DataBucket previousState = connectionData;

        //remove time-dependent variables
        DataBucket timeRemoved = current;
        timeRemoved.remove("robot_rtt");
        timeRemoved.remove("robotCpu");
        timeRemoved.remove("robotRam");

        DataBucket compressed = timeRemoved.compress(previousState);
        previousState = current;
        previousState.remove("robot_rtt");
        previousState.remove("robotCpu");
        previousState.remove("robotRam");
        connectionData = previousState;

        //skip if no changes
        if (compressed.toJson().empty()) return;

        //update time-dependent values and recompress
        DataBucket sentState = current;
        sentState["time"] = milliseconds_since_epoch;
        sentState["rtt"] = connectionState.rtt;
        sentState["controllerTime"] = controllerTime;
        compressed = sentState.compress(previousState);

        //update connection state
        connectionState.ready = false;
        connectionState.lastSend = robosub::Time::millis();

        //check if better to send as compressed or uncompressed3
        if (!compress || current.toString().length() < compressed.toString().length()) {
            //better to send uncompressed
            auto send_stream = make_shared<typename T::SendStream>();
            *send_stream << sentState;
            connection->send(send_stream);
        } else {
            //send compressed
            auto send_stream = make_shared<typename T::SendStream>();
            *send_stream << compressed;
            connection->send(send_stream);
        }
    } catch (exception e) {
        cerr << "Failed to send to robot: " << e.what() << endl;
    }
}

void network(ThreadData* threaddata) {
    DataBucket current = {};
    DataBucket toRobot = {};

    DataBucket clientConnectionData;
    ConnectionState clientConnectionState;
    bool clientConnected = false;

    WsClient client(threaddata->networkHost);
    client.on_open = [&clientConnectionState, &clientConnectionData, &toRobot, &clientConnected](
            shared_ptr<WsClient::Connection> connection) {
        clientConnected = true;
        clientConnectionData = toRobot;
        clientConnectionState = ConnectionState();
        clientConnectionState.lastSend = robosub::Time::millis();
		cout << "Client: Opened connection" << endl;
        //convert current state to stream
        auto send_stream = make_shared<WsClient::SendStream>();
        *send_stream << clientConnectionData;
        //send current state
        connection->send(send_stream, [](const robosub::ws::error_code &ec) {
            if (ec) {
				cerr << "Client: Error sending message. " <<
						"Error: " << ec << ", error message: " << ec.message() << endl;
            }
        });
    };

    client.on_message = [&current, &clientConnectionState](shared_ptr<WsClient::Connection> connection,
                                                           shared_ptr<WsClient::Message> message) {
        auto message_str = message->string();
        if (message_str == "\x06") {
            clientConnectionState.ready = true;
            clientConnectionState.rtt = robosub::Time::millis() - clientConnectionState.lastSend;
        } else {
            try {
                DataBucket temp = DataBucket(message_str);
                current["robot_rtt"] = temp["rtt"];
                current["pin"] = temp["pin"];
                current["robotCpu"] = Util::round<double>((double) temp["cpu"]);
                current["robotRam"] = Util::round<double>((double) temp["ram"]);
                current["imu"] = temp["imu"];
            } catch (exception e) {
                cerr << message_str << endl;
            }

            auto send_stream = make_shared<WsClient::SendStream>();
            *send_stream << "\x06";
            connection->send(send_stream);
        }
        //parse json into fromRobot data bucket (or maybe current?)
    };

    client.on_close = [&clientConnected](shared_ptr<WsClient::Connection> /*connection*/, int status,
                                         const string & /*reason*/) {
//		cout << "Client: Closed connection with status code " << status << endl;
        clientConnected = false;
    };

    // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html, Error Codes for error code meanings
    client.on_error = [&clientConnected](shared_ptr<WsClient::Connection> /*connection*/,
                                         const robosub::ws::error_code &ec) {
//		cout << "Client: Error: " << ec << ", error message: " << ec.message() << endl;
        clientConnected = false;
    };

    thread client_thread([&client]() {
        // Start WS-server
        while (true) {
            client.start();
			cout << "Not connected to robot" << endl;
            robosub::Time::waitMillis(1000);
        }
    });

    DataBucket previousState;

    int i = 0;
    while (true) {
        current["index"] = (i++ / 1000) % 1000; //force refresh approx every second
        current["robot_connected"] = clientConnected;

        try {
            threaddata->readout->rtt = current["robot_rtt"];
            threaddata->readout->cpu = current["robotCpu"];
            threaddata->readout->ram = current["robotRam"];

            if (current["imu"].is_object()) {
                threaddata->readout->accel_x = current["imu"]["ax"];
                threaddata->readout->accel_y = current["imu"]["ay"];
                threaddata->readout->accel_z = current["imu"]["az"];
            }

            threaddata->readout->valid = true;
        } catch (exception e) {
			cerr<<"exception setting telemetry: "<< e.what() <<endl;
            threaddata->readout->valid = false;
        }

        robosub::Time::waitMillis(1);

        unsigned long milliseconds_since_epoch = robosub::Time::millis();

        threaddata->controller1->controllerDataBucket(current, "controller1");
        threaddata->controller2->controllerDataBucket(current, "controller2");
        try {
            Mat x = robotState.motorValues(current["controller1"]["rx"],
                                           -(double) current["controller1"]["ly"],
                                           -(double) current["controller1"]["lx"]);
            toRobot["motors"]["ul"] = convertMotorValuesToRobot(x.at<double>(0, 0));
            toRobot["motors"]["ur"] = convertMotorValuesToRobot(x.at<double>(1, 0));
            toRobot["motors"]["bl"] = convertMotorValuesToRobot(x.at<double>(2, 0));
            toRobot["motors"]["br"] = convertMotorValuesToRobot(x.at<double>(3, 0));
            toRobot["motors"]["v"] = convertVerticalMotorValuesToRobot(-(double) current["controller1"]["ry"]);
            toRobot["motors"]["a1"] = convertArmValuesToRobot(-(double) current["controller2"]["ly"], 130);
            toRobot["motors"]["a2"] = convertArmRotationValuesToRobot((double) current["controller2"]["rx"], 20, 180);
            toRobot["motors"]["a3"] = int(std::max((double) current["controller2"]["t"], 0.0) * 65.0);
            toRobot["motors"]["a4"] = int((int) current["controller2"]["start"] * 2400);

        } catch (exception e) {
			cerr<<"exception reading motor values: "<<e.what()<<endl;
        }

        //send update to all active connections
        if (clientConnected)
            send<WsClient>(client.connection, clientConnectionState, clientConnectionData, toRobot,
                           milliseconds_since_epoch, false);


    }
}

#pragma clang diagnostic pop
