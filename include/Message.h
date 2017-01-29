#ifndef MESSAGE_H__
#define MESSAGE_H__

#include <string>

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

class LogMessage: public Message {
    public:
        // Log messages wrap the message envelope around a simple string.
        LogMessage(const std::string& s);
    protected:
        std::string str() const;
        std::string name() const;
    private:
        std::string message;
};


} // end (namespace robot)

#endif // (#ifndef MESSAGE_H__)
