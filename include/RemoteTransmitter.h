#ifndef REMOTE_TRANSMITTER_H__
#define REMOTE_TRANSMITTER_H__

#include "Config.h"

#include <thread>
#include <string>
#include <deque>

namespace robot {

// Base class for messages we'll be transmitting remotely.
//
// There are three types of messages we can transmit:
//
// (1) Vision calculations.
// (2) Log messages (including error messages.)
// (3) Heartbeat messages (these will ordinarily be discarded upon
//     receipt--they just let the receiver know that we're still online.)
//
// The first message type goes to the RoboRIO, if we can reach it.
//
// The second and third message types go to the driver station, if we can
// reach it.

class Message {
    public:

        // All messages can be converted to strings for transmission purposes.
        //
        // At present, we are using JSON for this, but that could change
        // later.
        operator std::string() const;

    protected:
        // When called, this function should return the (JSON) message payload
        // as a string.  An empty string is acceptable here if there is no
        // payload.
        //
        // Message::operator string() will take this payload and append a
        // timestamp and message type to it.

        virtual std::string str() const = 0;

        // This should return the name of the message type, such as
        // "heartbeat" or "log".
        virtual std::string name() const = 0;

    private:
        // Gets the current timestamp as a human-readable string (i.e., not
        // RFC 3339).
        std::string timestamp() const;
};

class HeartbeatMessage: public Message {
    public:
        // A heartbeat message doesn't contain any addition information beyond
        // its type.  The timestamp is added to it automatically.
        HeartbeatMessage();
    protected:
        std::string str() const;
        std::string name() const;
};




class RemoteTransmitter {
    public:

        // A remote transmitter needs to know where to transmit to.
        //
        // This function also starts our internal transmission thread.
        RemoteTransmitter(const Config& config);

        // Local stderr messages:

        // Shuts down the transmission queue.
        ~RemoteTransmitter();

        // Adds a message to the transmission queue for the robot.
        // TODO: Shouldn't this function only take arguments of type RobotMessage?
        //
        // This call does not block.
        void enqueueRobotMessage(const Message& message) const;

        // Adds a message to the transmission queue for the driver station.
        //
        // This call does not block.
        void enqueueDriverStationMessage(const Message& message) const;

    public:
        // The types that can be passed into RemoteTransmitter::logMessage().
        enum LogType {
            camera,
            sentToRobot,
            sentToDriverStation,
            cantSendToRobot,
            cantSendToDriverStation,
            debug
        };

        // Logs status messages to stderr.
        static void logMessage(LogType logType, const std::string& message);

    private:
        Config config_;
        std::thread transmissionThread;

        // The function executed by the transmission thread.
        static void threadFunction(const Config& config);

        static std::deque<std::string> robotTransmissionBuffer;
        static std::deque<std::string> driverStationTransmissionBuffer;
        static bool shutdown;  // If set to true, the thread will (eventually) end.
};


} // end (namespace robot)

#endif // (#ifndef REMOTE_TRANSMITTER_H__)
