#ifndef REMOTE_TRANSMITTER_H__
#define REMOTE_TRANSMITTER_H__

#include "Config.h"

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

        // Adds a message to the transmission queue.
        //
        // This call does not block.
        void enqueueMessage(const Message& message);

    private:
        Config config_;
        std::deque<std::string> transmissionBuffer;

        // Logs status messages to stderr.
        //
        // The message prefix indicates what happened:
        //
        //   [C --> *] Received new message from camera
        //   [* --> R] Transmitted message successfully to RoboRIO
        //   [* --> D] Transmitted message successfully to DriverStation
        //   [* -> R?] Can't reach RoboRIO
        //   [* -> D?] Can't reach DriverStation
        //   [ DEBUG ] Local stderr messages (mostly about the queue size)
        enum LogType {
            camera,
            sentToRobot,
            sentToDriverStation,
            cantSendToRobot,
            cantSendToDriverStation,
            debug
        };
        void logMessage(LogType logType, const std::string& message) const;

};


} // end (namespace robot)

#endif // (#ifndef REMOTE_TRANSMITTER_H__)
