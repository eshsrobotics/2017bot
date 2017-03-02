#include "RemoteTransmitter.h"

#include <iostream>
#include <stdexcept>

using std::ref;
using std::cerr;
using std::string;
using std::thread;
using std::exception;
using std::stringstream;

using namespace std::chrono;

namespace robot {

////////////////////////////////////
// Allocate static class objects. //
////////////////////////////////////

struct sigaction Connection::original_sigaction;
int Connection::connection_count;

//////////////////////////////////////
// TransmissionBuffer helper class. //
//////////////////////////////////////

////////////////////////////////////////////////////////////////
// The methods that actually perform the network connections. //
////////////////////////////////////////////////////////////////

///////////////////////////////////////
// RemoteTransmitter implementation. //
///////////////////////////////////////

// =========================================================================
// Construct the remote transmitter.

RemoteTransmitter::RemoteTransmitter(const Config& config, ConnectionPolicy connectionPolicy)
    : config_(config),
      connectionPolicy_(connectionPolicy),
      buffer_(),
      robotConnection_(buffer_),
      driverStationConnection_(buffer_),
      shutdown(false),
      // This starts the thread!
      transmissionThread(threadFunction,
                         ref(config_),
                         ref(connectionPolicy_),
                         ref(buffer_),
                         ref(robotConnection_),
                         ref(driverStationConnection_),
                         ref(shutdown)) { }

// =========================================================================
// Kill the remote transmitter.

RemoteTransmitter::~RemoteTransmitter() {
    // Tell the thread to shut down and block until that happens.
    shutdown = true;
    transmissionThread.join();
}

// =========================================================================
// Allow the caller to get or set our connection policy.

RemoteTransmitter::ConnectionPolicy RemoteTransmitter::connectionPolicy() const { return connectionPolicy_; }
void RemoteTransmitter::connectionPolicy(RemoteTransmitter::ConnectionPolicy connectionPolicy_) { this->connectionPolicy_ = connectionPolicy_; }

// =========================================================================
// Tell the caller what we're connected to.

Connection& RemoteTransmitter::robotConnection() { return robotConnection_; }
const Connection& RemoteTransmitter::robotConnection() const { return robotConnection_; }
Connection& RemoteTransmitter::driverStationConnection() { return driverStationConnection_; }
const Connection& RemoteTransmitter::driverStationConnection() const { return driverStationConnection_; }


// =========================================================================
// Give the caller something in which they can store messages for us to
// transmit.

TransmissionBuffer& RemoteTransmitter::buffer() { return buffer_; }


// =========================================================================
// Toss a message into the pile of stuff to transmit.

void RemoteTransmitter::enqueueRobotMessage(const CameraMessage& cameraMessage) {
    buffer_.robotQueue().push_back(static_cast<string>(cameraMessage));

    // The driver station will get a more condensed and easily-readable
    // message from mainLoop() itself.
    //
    // driverStationTransmissionBuffer.push_back(static_cast<string>(message));
}

void RemoteTransmitter::enqueueDriverStationMessage(const Message& message) {
    buffer_.driverStationQueue().push_back(static_cast<string>(message));
}


// =========================================================================
/// Connects to the robot and driver station and then transmits whatever
/// messages are in the queue for them.
///
/// @param config The Config that holds our known addresses, ports, and
///               timeouts for the roboRIO and the driver station.
///
/// @param connectionPolicy If the robotConnection cannot succeed in making a
///                         connection to the RoboRIO or the driver station,
///                         should we quit or should we keep trying?
///
/// @param buffer The TransmissionBuffer used for dequeuing the messages that
///               we will transmit.  Feel free to enqueue messages to it
///               asynchronously; we'll consume those as quickly as we can.
///
/// @param robotConnection[out] A Connection to wherever the Config says our
///                             RoboRIO is.  The caller should pass in a
///                             freshly-constructed object that is not
///                             connected to anything; we modify this object
///                             by attempting to make the connection (which
///                             might not succeed, mind you.)
///
///                             The caller can then use this updated object to
///                             perform queries about the connection even as
///                             we use the same object to write data to the
///                             remote server.
///
/// @param driverStationConnection[out] A Connection to wherever the Config
///                                     says the driver station is.  As usual,
///                                     you give us a freshly-constructed
///                                     Connection and we'll take care of
///                                     attempting to make it valid.
///
/// @param shutdown A reference to a boolean that will be set synchronously by
///                 the main thread.  We only continue to transmit while this
///                 value remains false.

void RemoteTransmitter::threadFunction(const Config& config,
                                       const ConnectionPolicy& connectionPolicy,
                                       TransmissionBuffer& buffer,
                                       Connection& robotConnection,
                                       Connection& driverStationConnection,
                                       const bool& shutdown) {

    // Transmit a heartbeat message when this many seconds have passed since
    // the last heartbeat message.
    const double heartbeatThresholdMilliseconds = 10000.0;
    auto lastHeartbeatTransmissionTime = high_resolution_clock::now();

    // Let's connect to the RoboRIO.
    buffer.logMessage(TransmissionBuffer::debug, "threadFunction: Opening connection to robot.");
    try {
        robotConnection.connect(config.robotAddresses(),
                                config.robotPort(),
                                config.robotTimeoutMilliseconds(),
                                TransmissionBuffer::cantSendToRobot);

    } catch (const exception& e) {
        buffer.logMessage(TransmissionBuffer::cantSendToRobot, "threadFunction: ERROR: Robot is unreachable.  Please check the addresses and port in the config file.");
    }


    // Let's connect to the driver station monitor.
    buffer.logMessage(TransmissionBuffer::debug, "threadFunction: Opening connection to driver station monitor.");
    try {

        driverStationConnection.connect(config.driverStationAddresses(),
                                        config.driverStationPort(),
                                        config.driverStationTimeoutMilliseconds(),
                                        TransmissionBuffer::cantSendToDriverStation);

    } catch(const exception& e) {
        buffer.logMessage(TransmissionBuffer::cantSendToDriverStation, "threadFunction: ERROR: Driver station is not reachable.  Please check the addresses and port in the config file.");
    }


    while (shutdown == false) {

        if (robotConnection.connected()) {

            // If there are messages in the queue, read one and transmit it.
            if (buffer.robotQueue().size() > 0) {
                string dataToTransmit = static_cast<string>(buffer.robotQueue().front());
                if (robotConnection.write(dataToTransmit, TransmissionBuffer::cantSendToRobot)) {
                    buffer.logMessage(TransmissionBuffer::sentToRobot, dataToTransmit);
                }
                buffer.robotQueue().pop_front();
            }

        } else if (connectionPolicy == AUTO_RECONNECT_ON_FAILURE) {

            try {
                robotConnection.reconnect();
            } catch(const exception& e) {
                buffer.logMessage(TransmissionBuffer::cantSendToRobot, "threadFunction: ERROR: Robot is still unreachable.  You really ought to check the addresses and port in the config file.");
            }
        }

        if (driverStationConnection.connected()) {

            // If there are messages in the queue, read one and transmit it.
            if (buffer.driverStationQueue().size() > 0) {
                string dataToTransmit = static_cast<string>(buffer.driverStationQueue().front());
                if (driverStationConnection.write(dataToTransmit, TransmissionBuffer::cantSendToDriverStation)) {
                    buffer.driverStationQueue().pop_front();
                }
            }

        } else if (connectionPolicy == AUTO_RECONNECT_ON_FAILURE) {

            try {
                driverStationConnection.reconnect();
            } catch(const exception& e) {
                buffer.logMessage(TransmissionBuffer::cantSendToDriverStation, "threadFunction: ERROR: Driver station is still unreachable.  You really ought to check the addresses and port in the config file.");
            }
        }


        // Send heartbeat messages every now and again.
        auto timeDelta = high_resolution_clock::now() - lastHeartbeatTransmissionTime;
        double elapsedMillisecondsSinceLastHeartbeat = duration<double, std::milli>(timeDelta).count();

        if (elapsedMillisecondsSinceLastHeartbeat > heartbeatThresholdMilliseconds) {
            buffer.driverStationQueue().push_back(static_cast<string>(HeartbeatMessage()));
            lastHeartbeatTransmissionTime = high_resolution_clock::now();
        }

        // Yield so this thread's not consuming 100% of a CPU core.
        std::this_thread::yield();

    } // end (main thread loop)

    cerr << "Shutdown detected.\n";
}

} // end (namespace robot)
